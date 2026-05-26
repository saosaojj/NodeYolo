import rclpy
from rclpy.node import Node
import yaml
import os
import json
from agv_interfaces.srv import GetConfig, SetConfig, SaveConfig, LoadConfig
from agv_interfaces.msg import SystemConfig, CameraConfig, PlcConfig


class ConfigManagerNode(Node):

    def __init__(self):
        super().__init__('config_manager')

        self.declare_parameter('config_path', '/workspace/agv_ros2_ws/src/agv_config/config/default_config.yaml')
        self.config_path = self.get_parameter('config_path').get_parameter_value().string_value

        self.config = {}
        self._load_default_config()

        self.get_config_srv = self.create_service(GetConfig, 'get_config', self.get_config_callback)
        self.set_config_srv = self.create_service(SetConfig, 'set_config', self.set_config_callback)
        self.save_config_srv = self.create_service(SaveConfig, 'save_config', self.save_config_callback)
        self.load_config_srv = self.create_service(LoadConfig, 'load_config', self.load_config_callback)

        self.config_publisher = self.create_publisher(SystemConfig, 'system_config', 10)
        self.get_logger().info('ConfigManagerNode initialized')
        self._publish_config()

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

    def _get_default_config(self):
        return {
            'system_name': 'AGV Control System',
            'version': '0.1.0',
            'auto_start': True,
            'log_level': 'INFO',
            'camera_configs': [],
            'plc_configs': []
        }

    def get_config_callback(self, request, response):
        try:
            if request.config_key == 'all':
                response.config_value = json.dumps(self.config)
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
            if '.' in request.config_key:
                keys = request.config_key.split('.')
                current = self.config
                for key in keys[:-1]:
                    if key not in current:
                        current[key] = {}
                    current = current[key]
                current[keys[-1]] = self._parse_value(request.config_value)
            else:
                self.config[request.config_key] = self._parse_value(request.config_value)
            response.success = True
            response.message = 'Config updated'
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
            path = request.file_path if request.file_path else self.config_path
            with open(path, 'w') as f:
                yaml.dump(self.config, f, default_flow_style=False)
            response.success = True
            response.message = f'Config saved to {path}'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def load_config_callback(self, request, response):
        try:
            path = request.file_path if request.file_path else self.config_path
            with open(path, 'r') as f:
                self.config = yaml.safe_load(f)
            response.success = True
            response.message = f'Config loaded from {path}'
            self._publish_config()
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

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
            self.get_logger().error(f'Failed to publish config: {e}')


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
