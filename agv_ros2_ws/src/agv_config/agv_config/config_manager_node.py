# 配置管理节点模块，负责AGV系统配置的加载、存储、查询和动态更新
import rclpy
from rclpy.node import Node
import yaml
import os
import json
from agv_interfaces.srv import GetConfig, SetConfig, SaveConfig, LoadConfig
from agv_interfaces.msg import SystemConfig, CameraConfig, PlcConfig


# 配置管理节点类，提供配置的增删改查服务，并通过话题广播系统配置
class ConfigManagerNode(Node):

    def __init__(self):
        super().__init__('config_manager')

        # 声明配置文件路径参数
        self.declare_parameter('config_path', '/workspace/agv_ros2_ws/src/agv_config/config/default_config.yaml')
        self.config_path = self.get_parameter('config_path').get_parameter_value().string_value

        self.config = {}
        # 加载默认配置
        self._load_default_config()

        # 创建配置管理相关的四个服务
        self.get_config_srv = self.create_service(GetConfig, 'get_config', self.get_config_callback)
        self.set_config_srv = self.create_service(SetConfig, 'set_config', self.set_config_callback)
        self.save_config_srv = self.create_service(SaveConfig, 'save_config', self.save_config_callback)
        self.load_config_srv = self.create_service(LoadConfig, 'load_config', self.load_config_callback)

        # 创建系统配置发布者
        self.config_publisher = self.create_publisher(SystemConfig, 'system_config', 10)
        self.get_logger().info('ConfigManagerNode initialized')
        # 初始化时发布一次配置
        self._publish_config()

    # 从YAML文件加载默认配置，文件不存在时使用内置默认值
    def _load_default_config(self):
        try:
            if os.path.exists(self.config_path):
                with open(self.config_path, 'r') as f:
                    self.config = yaml.safe_load(f)
                    self.get_logger().info(f'Loaded config from {self.config_path}')
            else:
                self.config = self._get_default_config()
                self.get_logger().warn(f'Config file not found, using defaults')
        except Exception as e:
            self.get_logger().error(f'Failed to load config: {e}')
            self.config = self._get_default_config()

    # 返回内置的默认配置字典
    def _get_default_config(self):
        return {
            'system_name': 'AGV Control System',
            'version': '0.1.0',
            'auto_start': True,
            'log_level': 'INFO',
            'camera_configs': [],
            'plc_configs': []
        }

    # 获取配置服务回调，支持查询全部配置、点分键路径查询和单键查询
    def get_config_callback(self, request, response):
        try:
            if request.config_key == 'all':
                # 查询全部配置，以JSON格式返回
                response.config_value = json.dumps(self.config)
            elif '.' in request.config_key:
                # 支持点分路径查询，如"camera_configs.0.camera_id"
                keys = request.config_key.split('.')
                value = self.config
                for key in keys:
                    value = value.get(key)
                    if value is None:
                        break
                response.config_value = str(value) if value is not None else ''
            else:
                # 单键查询
                response.config_value = str(self.config.get(request.config_key, ''))
            response.success = True
            response.message = 'Success'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    # 设置配置服务回调，支持点分键路径设置，更新后自动发布配置
    def set_config_callback(self, request, response):
        try:
            if '.' in request.config_key:
                # 支持点分路径设置嵌套配置
                keys = request.config_key.split('.')
                current = self.config
                for key in keys[:-1]:
                    if key not in current:
                        current[key] = {}
                    current = current[key]
                current[keys[-1]] = self._parse_value(request.config_value)
            else:
                # 单键设置
                self.config[request.config_key] = self._parse_value(request.config_value)
            response.success = True
            response.message = 'Config updated'
            # 配置更新后发布通知
            self._publish_config()
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    # 将字符串值解析为合适的Python类型（JSON、布尔、整数、浮点数或字符串）
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

    # 保存配置服务回调，将当前配置写入YAML文件
    def save_config_callback(self, request, response):
        try:
            path = request.file_path if request.file_path else self.config_path
            with open(path, 'w') as f:
                yaml.dump(self.config, f, default_flow_style=False)
            response.success = True
            response.message = f'Config saved to {path}'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    # 加载配置服务回调，从YAML文件重新加载配置
    def load_config_callback(self, request, response):
        try:
            path = request.file_path if request.file_path else self.config_path
            with open(path, 'r') as f:
                self.config = yaml.safe_load(f)
            response.success = True
            response.message = f'Config loaded from {path}'
            # 加载后发布配置通知
            self._publish_config()
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    # 将当前配置发布为SystemConfig消息，供其他节点订阅
    def _publish_config(self):
        try:
            msg = SystemConfig()
            msg.system_name = self.config.get('system_name', '')
            msg.version = self.config.get('version', '')
            msg.auto_start = self.config.get('auto_start', False)
            msg.log_level = self.config.get('log_level', 'INFO')

            # 构建摄像头配置消息列表
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

            # 构建PLC配置消息列表
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
            self.get_logger().error(f'Failed to publish config: {e}')


# 主函数，初始化ROS2并运行配置管理节点
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
