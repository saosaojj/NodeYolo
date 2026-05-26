# YOLO模型训练节点模块，提供ROS2服务接口用于在线训练自定义YOLO模型
import os
import threading
import time

from rclpy.node import Node
from std_msgs.msg import String
from ultralytics import YOLO

from agv_interfaces.srv import TrainModel


# YOLO模型训练ROS2节点，支持异步训练、GPU显存限制和早停机制
class YoloTrainerNode(Node):

    def __init__(self):
        super().__init__('yolo_trainer')

        # 声明ROS2参数：默认训练轮数、学习率、模型类型、图像尺寸、数据集路径等
        self.declare_parameter('default_epochs', 100)
        self.declare_parameter('default_learning_rate', 0.01)
        self.declare_parameter('default_model_type', 'yolov8n')
        self.declare_parameter('default_imgsz', 640)
        self.declare_parameter('dataset_base_path', '/tmp/agv_datasets')
        self.declare_parameter('model_output_path', '/tmp/agv_models')
        self.declare_parameter('early_stopping_patience', 50)
        self.declare_parameter('gpu_memory_limit_fraction', 0.8)
        self.declare_parameter('checkpoint_interval', 10)

        # 获取参数值
        self.default_epochs = self.get_parameter('default_epochs').get_parameter_value().integer_value
        self.default_learning_rate = self.get_parameter('default_learning_rate').get_parameter_value().double_value
        self.default_model_type = self.get_parameter('default_model_type').get_parameter_value().string_value
        self.default_imgsz = self.get_parameter('default_imgsz').get_parameter_value().integer_value
        self.model_output_path = self.get_parameter('model_output_path').get_parameter_value().string_value
        self._early_stopping_patience = self.get_parameter('early_stopping_patience').get_parameter_value().integer_value
        self._gpu_memory_limit_fraction = self.get_parameter('gpu_memory_limit_fraction').get_parameter_value().double_value
        self._checkpoint_interval = self.get_parameter('checkpoint_interval').get_parameter_value().integer_value

        # 训练状态控制变量
        self._training_active = False
        self._training_lock = threading.Lock()
        self._cancel_requested = False

        # 创建训练状态话题发布者和训练服务
        self.status_pub = self.create_publisher(String, 'training_status', 10)

        self.train_srv = self.create_service(
            TrainModel,
            'train_model',
            self.train_model_callback
        )

        self.get_logger().info('YoloTrainerNode initialized')

    # 验证训练请求参数的合法性
    def _validate_training_request(self, dataset_path, data_yaml, model_type, epochs, learning_rate):
        if not os.path.exists(dataset_path):
            return False, f'Dataset path does not exist: {dataset_path}'
        if not os.path.exists(data_yaml):
            return False, f'data.yaml not found in dataset path: {dataset_path}'
        if not model_type:
            return False, 'Model type must be specified'
        if epochs <= 0:
            return False, 'Epochs must be positive'
        if learning_rate <= 0.0:
            return False, 'Learning rate must be positive'
        # 检查data.yaml文件是否为空
        try:
            with open(data_yaml, 'r') as f:
                content = f.read().strip()
                if not content:
                    return False, 'data.yaml is empty'
        except Exception as e:
            return False, f'Cannot read data.yaml: {e}'
        return True, ''

    # 训练服务回调，验证请求参数后启动异步训练线程
    def train_model_callback(self, request, response):
        with self._training_lock:
            # 同一时间只允许一个训练任务
            if self._training_active:
                response.success = False
                response.message = 'Training is already in progress'
                response.model_path = ''
                response.training_time = 0.0
                return response
            self._training_active = True
            self._cancel_requested = False

        # 使用请求参数或默认值
        dataset_path = request.dataset_path
        model_type = request.model_type if request.model_type else self.default_model_type
        epochs = request.epochs if request.epochs > 0 else self.default_epochs
        learning_rate = request.learning_rate if request.learning_rate > 0.0 else self.default_learning_rate
        output_path = request.output_path if request.output_path else self.model_output_path

        data_yaml = os.path.join(dataset_path, 'data.yaml')

        # 验证训练请求参数
        valid, msg = self._validate_training_request(dataset_path, data_yaml, model_type, epochs, learning_rate)
        if not valid:
            with self._training_lock:
                self._training_active = False
            response.success = False
            response.message = msg
            response.model_path = ''
            response.training_time = 0.0
            return response

        os.makedirs(output_path, exist_ok=True)

        # 在独立线程中执行训练，避免阻塞ROS2服务回调
        thread = threading.Thread(
            target=self._run_training,
            args=(data_yaml, model_type, epochs, learning_rate, output_path),
            daemon=True
        )
        thread.start()

        response.success = True
        response.message = 'Training started'
        response.model_path = ''
        response.training_time = 0.0
        return response

    # 设置GPU显存使用上限，防止训练占用过多显存影响其他节点
    def _apply_gpu_memory_limit(self):
        try:
            import torch
            if torch.cuda.is_available():
                fraction = self._gpu_memory_limit_fraction
                torch.cuda.set_per_process_memory_fraction(fraction, 0)
                self.get_logger().info(f'GPU memory limit set to {fraction * 100:.0f}%')
        except ImportError:
            pass
        except Exception as e:
            self.get_logger().warn(f'Could not set GPU memory limit: {e}')

    # 训练执行函数，加载基础模型并调用YOLO训练API
    def _run_training(self, data_yaml, model_type, epochs, learning_rate, output_path):
        start_time = time.time()
        self._publish_status('Training started')

        try:
            self._apply_gpu_memory_limit()

            # 加载预训练基础模型
            self._publish_status(f'Loading base model: {model_type}')
            model = YOLO(f'{model_type}.pt')

            self._publish_status(
                f'Training: epochs={epochs}, lr={learning_rate}, data={data_yaml}'
            )

            # 调用YOLO训练API，配置早停和检查点保存
            results = model.train(
                data=data_yaml,
                epochs=epochs,
                lr0=learning_rate,
                imgsz=self.default_imgsz,
                project=output_path,
                name='train',
                exist_ok=True,
                verbose=True,
                patience=self._early_stopping_patience,
                save_period=self._checkpoint_interval,
            )

            elapsed = time.time() - start_time

            # 检查训练输出的最佳模型权重文件
            best_weight_dir = os.path.join(output_path, 'train', 'weights')
            best_model_path = os.path.join(best_weight_dir, 'best.pt')

            if os.path.exists(best_model_path):
                self._publish_status(
                    f'Training completed in {elapsed:.1f}s. Model saved: {best_model_path}'
                )
            else:
                self._publish_status(
                    f'Training completed in {elapsed:.1f}s. Check output directory for weights.'
                )

        except Exception as e:
            elapsed = time.time() - start_time
            self._publish_status(f'Training failed: {e}')
            self.get_logger().error(f'Training failed: {e}')

        finally:
            with self._training_lock:
                self._training_active = False

    # 发布训练状态消息到ROS2话题
    def _publish_status(self, message):
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)
        self.get_logger().info(message)


# 节点入口函数，初始化ROS2并启动YOLO训练节点
def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = YoloTrainerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
