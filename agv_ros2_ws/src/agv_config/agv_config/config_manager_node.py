import copy
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy_interfaces.msg import SetParametersResult
import yaml
import os
import json
import time
from std_msgs.msg import String, Int32
from std_srvs.srv import Trigger
from agv_interfaces.srv import GetConfig, SetConfig, SaveConfig, LoadConfig
from agv_interfaces.msg import SystemConfig, CameraConfig, PlcConfig


CONFIG_SCHEMA = {
    'system_name': {'type': str, 'required': True, 'min_length': 1, 'max_length': 256},
    'version': {'type': str, 'required': True, 'pattern': r'^\d+\.\d+\.\d+$'},
    'auto_start': {'type': bool, 'required': True},
    'log_level': {'type': str, 'required': True, 'allowed': ['DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL']},
    'camera_configs': {'type': list, 'required': False},
    'plc_configs': {'type': list, 'required': False},
}


class ConfigManagerNode(Node):

    def __init__(self):
        super().__init__('config_manager')

        self.declare_parameter('config_path', '/workspace/agv_ros2_ws/src/agv_config/config/default_config.yaml')
        self.declare_parameter('health_check_interval', 30.0)
        self.declare_parameter('max_history_versions', 10)

        self.config_path = self.get_parameter('config_path').get_parameter_value().string_value
        self._max_history = self.get_parameter('max_history_versions').get_parameter_value().integer_value

        # 配置版本管理
        self._config_version = 0
        # 配置历史记录，用于回滚
        self._config_history = []
        # 当前配置
        self.config = {}
        self._load_default_config()
        # 保存初始版本到历史
        self._save_to_history()

        # 原有服务
        self.get_config_srv = self.create_service(GetConfig, 'get_config', self.get_config_callback)
        self.set_config_srv = self.create_service(SetConfig, 'set_config', self.set_config_callback)
        self.save_config_srv = self.create_service(SaveConfig, 'save_config', self.save_config_callback)
        self.load_config_srv = self.create_service(LoadConfig, 'load_config', self.load_config_callback)

        # 新增服务：回滚配置
        self.rollback_srv = self.create_service(Trigger, 'rollback_config', self.rollback_config_callback)
        # 新增服务：恢复出厂默认配置
        self.reset_defaults_srv = self.create_service(Trigger, 'reset_to_defaults', self.reset_to_defaults_callback)
        # 新增服务：导出配置为JSON
        self.export_config_srv = self.create_service(GetConfig, 'export_config', self.export_config_callback)
        # 新增服务：从JSON导入配置
        self.import_config_srv = self.create_service(SetConfig, 'import_config', self.import_config_callback)

        # 原有发布者
        self.config_publisher = self.create_publisher(SystemConfig, 'system_config', 10)

        # 新增发布者：配置变更通知
        self.config_changed_pub = self.create_publisher(String, 'config_changed', 10)
        # 新增发布者：配置版本号
        self.config_version_pub = self.create_publisher(Int32, 'config_version', 10)
        # 新增发布者：配置健康状态
        self.config_health_pub = self.create_publisher(String, 'config_health', 10)

        # 参数变更监控回调
        self.add_on_set_parameters_callback(self._on_parameter_change)

        # 配置健康检查定时器
        health_interval = self.get_parameter('health_check_interval').get_parameter_value().double_value
        self._health_timer = self.create_timer(health_interval, self._health_check_callback)

        self.get_logger().info('ConfigManagerNode已初始化，支持配置验证、版本管理、回滚、导入导出、健康检查')
        self._publish_config()
        self._publish_version()

    def _load_default_config(self):
        try:
            if os.path.exists(self.config_path):
                with open(self.config_path, 'r') as f:
                    self.config = yaml.safe_load(f)
                    self.get_logger().info(f'从 {self.config_path} 加载配置')
            else:
                self.config = self._get_default_config()
                self.get_logger().warn('配置文件未找到，使用默认配置')
        except Exception as e:
            self.get_logger().error(f'加载配置失败: {e}')
            self.config = self._get_default_config()

    def _get_default_config(self):
        return {
            'system_name': 'AGV Control System',
            'version': '0.1.0',
            'auto_start': True,
            'log_level': 'INFO',
            'camera_configs': [],
            'plc_configs': []
        }

    # ==================== 配置验证 ====================

    def _validate_config(self, config):
        """验证配置值是否符合schema定义"""
        errors = []
        if not isinstance(config, dict):
            return False, ['配置必须是字典类型']

        for key, schema in CONFIG_SCHEMA.items():
            if schema.get('required', False) and key not in config:
                errors.append(f'缺少必填字段: {key}')
                continue

            if key not in config:
                continue

            value = config[key]
            expected_type = schema.get('type')
            if expected_type and not isinstance(value, expected_type):
                errors.append(f'字段 {key} 类型错误: 期望 {expected_type.__name__}, 实际 {type(value).__name__}')
                continue

            if 'min_length' in schema and isinstance(value, str) and len(value) < schema['min_length']:
                errors.append(f'字段 {key} 长度不足: 最小 {schema["min_length"]}')

            if 'max_length' in schema and isinstance(value, str) and len(value) > schema['max_length']:
                errors.append(f'字段 {key} 长度超限: 最大 {schema["max_length"]}')

            if 'allowed' in schema and value not in schema['allowed']:
                errors.append(f'字段 {key} 值无效: {value}, 允许值: {schema["allowed"]}')

            if 'pattern' in schema and isinstance(value, str):
                import re
                if not re.match(schema['pattern'], value):
                    errors.append(f'字段 {key} 格式不匹配: {value}, 要求: {schema["pattern"]}')

        # 验证camera_configs子项
        for i, cam in enumerate(config.get('camera_configs', [])):
            if not isinstance(cam, dict):
                errors.append(f'camera_configs[{i}] 必须是字典类型')
                continue
            if 'camera_id' not in cam:
                errors.append(f'camera_configs[{i}] 缺少 camera_id')
            if 'width' in cam and (cam['width'] < 1 or cam['width'] > 4096):
                errors.append(f'camera_configs[{i}] width 超出范围(1-4096)')
            if 'height' in cam and (cam['height'] < 1 or cam['height'] > 4096):
                errors.append(f'camera_configs[{i}] height 超出范围(1-4096)')
            if 'fps' in cam and (cam['fps'] < 1 or cam['fps'] > 120):
                errors.append(f'camera_configs[{i}] fps 超出范围(1-120)')

        # 验证plc_configs子项
        for i, plc in enumerate(config.get('plc_configs', [])):
            if not isinstance(plc, dict):
                errors.append(f'plc_configs[{i}] 必须是字典类型')
                continue
            if 'plc_id' not in plc:
                errors.append(f'plc_configs[{i}] 缺少 plc_id')
            if 'port' in plc and (plc['port'] < 1 or plc['port'] > 65535):
                errors.append(f'plc_configs[{i}] port 超出范围(1-65535)')
            if 'timeout' in plc and plc['timeout'] <= 0:
                errors.append(f'plc_configs[{i}] timeout 必须大于0')

        is_valid = len(errors) == 0
        return is_valid, errors

    def _validate_single_key(self, key, value):
        """验证单个配置键值"""
        if key in CONFIG_SCHEMA:
            schema = CONFIG_SCHEMA[key]
            expected_type = schema.get('type')
            if expected_type and not isinstance(value, expected_type):
                return False, f'字段 {key} 类型错误: 期望 {expected_type.__name__}, 实际 {type(value).__name__}'
            if 'min_length' in schema and isinstance(value, str) and len(value) < schema['min_length']:
                return False, f'字段 {key} 长度不足'
            if 'max_length' in schema and isinstance(value, str) and len(value) > schema['max_length']:
                return False, f'字段 {key} 长度超限'
            if 'allowed' in schema and value not in schema['allowed']:
                return False, f'字段 {key} 值无效: {value}'
            if 'pattern' in schema and isinstance(value, str):
                import re
                if not re.match(schema['pattern'], value):
                    return False, f'字段 {key} 格式不匹配'
        return True, ''

    # ==================== 版本管理 ====================

    def _save_to_history(self):
        """保存当前配置快照到历史记录"""
        snapshot = copy.deepcopy(self.config)
        self._config_history.append({
            'version': self._config_version,
            'config': snapshot,
            'timestamp': time.time()
        })
        # 限制历史记录数量
        while len(self._config_history) > self._max_history:
            self._config_history.pop(0)

    def _increment_version(self):
        """递增配置版本号并发布"""
        self._config_version += 1
        self._publish_version()

    def _publish_version(self):
        """发布当前配置版本号"""
        msg = Int32()
        msg.data = self._config_version
        self.config_version_pub.publish(msg)

    # ==================== 变更通知 ====================

    def _notify_config_change(self, key, old_value, new_value):
        """发布配置变更通知"""
        change_info = {
            'key': key,
            'old_value': old_value,
            'new_value': new_value,
            'version': self._config_version,
            'timestamp': time.time()
        }
        msg = String()
        msg.data = json.dumps(change_info, ensure_ascii=False, default=str)
        self.config_changed_pub.publish(msg)
        self.get_logger().info(f'配置变更: {key}, 版本: {self._config_version}')

    # ==================== 参数变更监控 ====================

    def _on_parameter_change(self, params):
        """ROS2参数变更回调，监控运行时参数修改"""
        for param in params:
            if param.name == 'config_path':
                self.get_logger().info(f'检测到参数变更: config_path = {param.value}')
                new_path = param.value
                if os.path.exists(new_path):
                    self.config_path = new_path
                    self.get_logger().info(f'配置路径已更新为: {new_path}')
                else:
                    self.get_logger().warn(f'参数变更的配置路径不存在: {new_path}')
            elif param.name == 'health_check_interval':
                self.get_logger().info(f'检测到参数变更: health_check_interval = {param.value}')
            elif param.name == 'max_history_versions':
                self.get_logger().info(f'检测到参数变更: max_history_versions = {param.value}')
                self._max_history = param.value
        return SetParametersResult(successful=True)

    # ==================== 健康检查 ====================

    def _health_check_callback(self):
        """定期配置健康检查，验证所有配置值并发布健康状态"""
        is_valid, errors = self._validate_config(self.config)
        health_info = {
            'healthy': is_valid,
            'version': self._config_version,
            'timestamp': time.time(),
            'config_path': self.config_path,
            'config_path_exists': os.path.exists(self.config_path),
            'errors': errors if not is_valid else [],
            'total_keys': len(self.config),
            'history_depth': len(self._config_history)
        }
        msg = String()
        msg.data = json.dumps(health_info, ensure_ascii=False)
        self.config_health_pub.publish(msg)

        if not is_valid:
            self.get_logger().warn(f'配置健康检查发现问题: {errors}')

    # ==================== 原有服务回调（增强） ====================

    def get_config_callback(self, request, response):
        try:
            if request.config_key == 'all':
                response.config_value = json.dumps(self.config, ensure_ascii=False, default=str)
            elif '.' in request.config_key:
                keys = request.config_key.split('.')
                value = self.config
                for key in keys:
                    value = value.get(key)
                    if value is None:
                        break
                response.config_value = str(value) if value is not None else ''
            else:
                response.config_value = str(self.config.get(request.config_key, ''))
            response.success = True
            response.message = 'Success'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def set_config_callback(self, request, response):
        try:
            parsed_value = self._parse_value(request.config_value)

            # 获取旧值用于变更通知
            old_value = None
            if '.' in request.config_key:
                keys = request.config_key.split('.')
                current = self.config
                for key in keys[:-1]:
                    current = current.get(key, {})
                old_value = current.get(keys[-1])
            else:
                old_value = self.config.get(request.config_key)

            # 验证单个键值
            is_valid, err_msg = self._validate_single_key(request.config_key, parsed_value)
            if not is_valid:
                response.success = False
                response.message = f'配置验证失败: {err_msg}'
                return response

            # 保存变更前快照
            self._save_to_history()

            # 应用变更
            if '.' in request.config_key:
                keys = request.config_key.split('.')
                current = self.config
                for key in keys[:-1]:
                    if key not in current:
                        current[key] = {}
                    current = current[key]
                current[keys[-1]] = parsed_value
            else:
                self.config[request.config_key] = parsed_value

            # 递增版本号
            self._increment_version()

            # 发布变更通知
            self._notify_config_change(request.config_key, old_value, parsed_value)

            response.success = True
            response.message = f'配置已更新, 版本: {self._config_version}'
            self._publish_config()
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _parse_value(self, value_str):
        try:
            return json.loads(value_str)
        except:
            if value_str.lower() == 'true':
                return True
            elif value_str.lower() == 'false':
                return False
            try:
                return int(value_str)
            except:
                try:
                    return float(value_str)
                except:
                    return value_str

    def save_config_callback(self, request, response):
        try:
            # 保存前验证配置
            is_valid, errors = self._validate_config(self.config)
            if not is_valid:
                response.success = False
                response.message = f'配置验证失败，无法保存: {errors}'
                return response

            path = request.file_path if request.file_path else self.config_path
            with open(path, 'w') as f:
                yaml.dump(self.config, f, default_flow_style=False)
            response.success = True
            response.message = f'配置已保存到 {path}, 版本: {self._config_version}'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def load_config_callback(self, request, response):
        try:
            path = request.file_path if request.file_path else self.config_path
            with open(path, 'r') as f:
                new_config = yaml.safe_load(f)

            # 加载前验证配置
            is_valid, errors = self._validate_config(new_config)
            if not is_valid:
                response.success = False
                response.message = f'加载的配置验证失败: {errors}'
                return response

            # 保存当前配置到历史
            self._save_to_history()

            old_config = self.config
            self.config = new_config

            # 递增版本号
            self._increment_version()

            # 发布所有顶层键的变更通知
            all_keys = set(list(old_config.keys()) + list(self.config.keys()))
            for key in all_keys:
                old_val = old_config.get(key)
                new_val = self.config.get(key)
                if old_val != new_val:
                    self._notify_config_change(key, old_val, new_val)

            response.success = True
            response.message = f'配置已从 {path} 加载, 版本: {self._config_version}'
            self._publish_config()
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    # ==================== 新增服务回调 ====================

    def rollback_config_callback(self, request, response):
        """回滚到上一个配置版本"""
        try:
            if len(self._config_history) < 2:
                response.success = False
                response.message = '没有足够的配置历史记录用于回滚'
                return response

            # 弹出当前版本的历史记录
            current_snapshot = self._config_history.pop()
            # 获取上一个版本
            previous = self._config_history[-1]

            old_config = self.config
            self.config = copy.deepcopy(previous['config'])
            self._config_version = previous['version']

            # 发布变更通知
            all_keys = set(list(old_config.keys()) + list(self.config.keys()))
            for key in all_keys:
                old_val = old_config.get(key)
                new_val = self.config.get(key)
                if old_val != new_val:
                    self._notify_config_change(key, old_val, new_val)

            self._publish_version()
            self._publish_config()

            response.success = True
            response.message = f'已回滚到版本 {previous["version"]}'
            self.get_logger().info(f'配置已回滚到版本 {previous["version"]}')
        except Exception as e:
            response.success = False
            response.message = f'回滚失败: {str(e)}'
        return response

    def reset_to_defaults_callback(self, request, response):
        """恢复出厂默认配置"""
        try:
            # 保存当前配置到历史
            self._save_to_history()

            old_config = self.config
            self.config = self._get_default_config()

            # 递增版本号
            self._increment_version()

            # 发布变更通知
            all_keys = set(list(old_config.keys()) + list(self.config.keys()))
            for key in all_keys:
                old_val = old_config.get(key)
                new_val = self.config.get(key)
                if old_val != new_val:
                    self._notify_config_change(key, old_val, new_val)

            self._publish_config()

            response.success = True
            response.message = f'已恢复出厂默认配置, 版本: {self._config_version}'
            self.get_logger().info('配置已恢复为出厂默认值')
        except Exception as e:
            response.success = False
            response.message = f'恢复默认配置失败: {str(e)}'
        return response

    def export_config_callback(self, request, response):
        """导出配置为JSON字符串"""
        try:
            export_data = {
                'config': self.config,
                'version': self._config_version,
                'export_time': time.time(),
                'config_path': self.config_path
            }
            response.config_value = json.dumps(export_data, ensure_ascii=False, default=str)
            response.success = True
            response.message = f'配置已导出, 版本: {self._config_version}'
        except Exception as e:
            response.success = False
            response.config_value = ''
            response.message = f'导出失败: {str(e)}'
        return response

    def import_config_callback(self, request, response):
        """从JSON字符串导入配置"""
        try:
            import_data = json.loads(request.config_value)

            # 支持两种格式：纯配置字典 或 包含元数据的导出格式
            if 'config' in import_data and isinstance(import_data['config'], dict):
                new_config = import_data['config']
            else:
                new_config = import_data

            # 验证导入的配置
            is_valid, errors = self._validate_config(new_config)
            if not is_valid:
                response.success = False
                response.message = f'导入配置验证失败: {errors}'
                return response

            # 保存当前配置到历史
            self._save_to_history()

            old_config = self.config
            self.config = new_config

            # 递增版本号
            self._increment_version()

            # 发布变更通知
            all_keys = set(list(old_config.keys()) + list(self.config.keys()))
            for key in all_keys:
                old_val = old_config.get(key)
                new_val = self.config.get(key)
                if old_val != new_val:
                    self._notify_config_change(key, old_val, new_val)

            self._publish_config()

            response.success = True
            response.message = f'配置已导入, 版本: {self._config_version}'
            self.get_logger().info(f'配置已从JSON导入, 版本: {self._config_version}')
        except json.JSONDecodeError as e:
            response.success = False
            response.message = f'JSON解析失败: {str(e)}'
        except Exception as e:
            response.success = False
            response.message = f'导入失败: {str(e)}'
        return response

    # ==================== 配置发布 ====================

    def _publish_config(self):
        try:
            msg = SystemConfig()
            msg.system_name = self.config.get('system_name', '')
            msg.version = self.config.get('version', '')
            msg.auto_start = self.config.get('auto_start', False)
            msg.log_level = self.config.get('log_level', 'INFO')

            for cam_cfg in self.config.get('camera_configs', []):
                cam_msg = CameraConfig()
                cam_msg.camera_id = cam_cfg.get('camera_id', '')
                cam_msg.device_path = cam_cfg.get('device_path', '')
                cam_msg.width = cam_cfg.get('width', 640)
                cam_msg.height = cam_cfg.get('height', 480)
                cam_msg.fps = cam_cfg.get('fps', 30)
                cam_msg.format = cam_cfg.get('format', 'bgr8')
                cam_msg.exposure_mode = cam_cfg.get('exposure_mode', 'auto')
                cam_msg.exposure = cam_cfg.get('exposure', 0.0)
                cam_msg.gain = cam_cfg.get('gain', 0.0)
                cam_msg.auto_exposure = cam_cfg.get('auto_exposure', True)
                cam_msg.enabled = cam_cfg.get('enabled', True)
                msg.camera_configs.append(cam_msg)

            for plc_cfg in self.config.get('plc_configs', []):
                plc_msg = PlcConfig()
                plc_msg.plc_id = plc_cfg.get('plc_id', '')
                plc_msg.ip_address = plc_cfg.get('ip_address', '')
                plc_msg.port = plc_cfg.get('port', 502)
                plc_msg.slave_id = plc_cfg.get('slave_id', 1)
                plc_msg.timeout = plc_cfg.get('timeout', 5)
                plc_msg.enabled = plc_cfg.get('enabled', True)
                msg.plc_configs.append(plc_msg)

            self.config_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'发布配置失败: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ConfigManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
