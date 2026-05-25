import os
import threading
import time

from rclpy.node import Node
from std_msgs.msg import String
from ultralytics import YOLO

from agv_interfaces.srv import TrainModel


class YoloTrainerNode(Node):

    def __init__(self):
        super().__init__('yolo_trainer')

        self.declare_parameter('default_epochs', 100)
        self.declare_parameter('default_learning_rate', 0.01)
        self.declare_parameter('default_model_type', 'yolov8n')
        self.declare_parameter('default_imgsz', 640)
        self.declare_parameter('dataset_base_path', '/tmp/agv_datasets')
        self.declare_parameter('model_output_path', '/tmp/agv_models')

        self.default_epochs = self.get_parameter('default_epochs').get_parameter_value().integer_value
        self.default_learning_rate = self.get_parameter('default_learning_rate').get_parameter_value().double_value
        self.default_model_type = self.get_parameter('default_model_type').get_parameter_value().string_value
        self.default_imgsz = self.get_parameter('default_imgsz').get_parameter_value().integer_value
        self.model_output_path = self.get_parameter('model_output_path').get_parameter_value().string_value

        self._training_active = False
        self._training_lock = threading.Lock()

        self.status_pub = self.create_publisher(String, 'training_status', 10)

        self.train_srv = self.create_service(
            TrainModel,
            'train_model',
            self.train_model_callback
        )

        self.get_logger().info('YoloTrainerNode initialized')

    def train_model_callback(self, request, response):
        with self._training_lock:
            if self._training_active:
                response.success = False
                response.message = 'Training is already in progress'
                response.model_path = ''
                response.training_time = 0.0
                return response
            self._training_active = True

        dataset_path = request.dataset_path
        model_type = request.model_type if request.model_type else self.default_model_type
        epochs = request.epochs if request.epochs > 0 else self.default_epochs
        learning_rate = request.learning_rate if request.learning_rate > 0.0 else self.default_learning_rate
        output_path = request.output_path if request.output_path else self.model_output_path

        if not os.path.exists(dataset_path):
            with self._training_lock:
                self._training_active = False
            response.success = False
            response.message = f'Dataset path does not exist: {dataset_path}'
            response.model_path = ''
            response.training_time = 0.0
            return response

        data_yaml = os.path.join(dataset_path, 'data.yaml')
        if not os.path.exists(data_yaml):
            with self._training_lock:
                self._training_active = False
            response.success = False
            response.message = f'data.yaml not found in dataset path: {dataset_path}'
            response.model_path = ''
            response.training_time = 0.0
            return response

        os.makedirs(output_path, exist_ok=True)

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

    def _run_training(self, data_yaml, model_type, epochs, learning_rate, output_path):
        start_time = time.time()
        self._publish_status('Training started')

        try:
            self._publish_status(f'Loading base model: {model_type}')
            model = YOLO(f'{model_type}.pt')

            self._publish_status(
                f'Training: epochs={epochs}, lr={learning_rate}, data={data_yaml}'
            )

            results = model.train(
                data=data_yaml,
                epochs=epochs,
                lr0=learning_rate,
                imgsz=self.default_imgsz,
                project=output_path,
                name='train',
                exist_ok=True,
                verbose=True
            )

            elapsed = time.time() - start_time

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

    def _publish_status(self, message):
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)
        self.get_logger().info(message)


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
