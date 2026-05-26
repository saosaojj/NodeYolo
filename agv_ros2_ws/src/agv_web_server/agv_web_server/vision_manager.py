#!/usr/bin/env python3
"""
视觉管理器模块

集成 supervision 视觉工具箱和 YOLO 模型，
提供目标检测、结果分析、可视化标注、区域统计、越线检测等高级视觉功能。
支持模型自训练和参数微调。
"""

import os
import threading
import time
import json
from pathlib import Path

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

try:
    import numpy as np
    NUMPY_AVAILABLE = True
except ImportError:
    NUMPY_AVAILABLE = False

try:
    import supervision as sv
    SUPERVISION_AVAILABLE = True
except ImportError:
    SUPERVISION_AVAILABLE = False

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


class VisionManager:
    """
    视觉管理器
    
    集成 supervision 视觉工具箱，提供：
    - YOLO 模型推理和结果分析
    - 高级可视化标注（边界框、标签、掩码、轨迹）
    - 区域统计和越线检测
    - 目标跟踪和轨迹平滑
    - 切片推理（大图处理）
    - 模型自训练和参数微调
    """

    _instance = None
    _instance_lock = threading.Lock()

    @classmethod
    def get_instance(cls):
        """获取 VisionManager 单例实例"""
        if cls._instance is None:
            with cls._instance_lock:
                if cls._instance is None:
                    cls._instance = cls()
        return cls._instance

    def __init__(self):
        """
        初始化视觉管理器
        """
        self._lock = threading.Lock()
        self._simulation_enabled = False

        # 模型相关
        self._model = None
        self._model_path = 'yolov8n.pt'
        self._model_name = 'yolov8n'
        self._confidence = 0.5
        self._iou_threshold = 0.45
        self._device = 'cpu'
        self._classes_filter = []
        self._imgsz = 640

        # supervision 标注器
        self._box_annotator = None
        self._label_annotator = None
        self._mask_annotator = None
        self._trace_annotator = None
        self._bounding_box_annotator = None
        self._round_box_annotator = None
        self._color_annotator = None

        # 区域和越线检测
        self._zones = {}
        self._zone_annotators = {}
        self._line_zones = {}
        self._line_zone_annotators = {}

        # 跟踪器
        self._tracker = None
        self._track_history = {}

        # 切片推理
        self._sahi_model = None
        self._sahi_enabled = False

        # 训练状态
        self._training_active = False
        self._training_progress = {}
        self._training_lock = threading.Lock()
        self._cancel_training = False

        # 数据集管理
        self._dataset_path = '/tmp/agv_datasets'
        self._model_output_path = '/tmp/agv_models'

        # 初始化标注器
        self._init_annotators()

        # 从数据库加载仿真状态
        self._load_simulation_state()

    def _init_annotators(self):
        """初始化 supervision 标注器"""
        if not SUPERVISION_AVAILABLE:
            return

        try:
            self._box_annotator = sv.BoxAnnotator()
            self._label_annotator = sv.LabelAnnotator()
            self._bounding_box_annotator = sv.BoundingBoxAnnotator()
            self._round_box_annotator = sv.RoundBoxAnnotator()
        except Exception:
            pass

        try:
            self._mask_annotator = sv.MaskAnnotator()
        except Exception:
            pass

        try:
            self._trace_annotator = sv.TraceAnnotator()
        except Exception:
            pass

        try:
            self._color_annotator = sv.ColorAnnotator()
        except Exception:
            pass

    def _load_simulation_state(self):
        """从数据库加载仿真状态"""
        try:
            from agv_web_server.database_manager import DatabaseManager
            db = DatabaseManager()
            state = db.get_simulation_state()
            if state and 'vision' in state:
                self._simulation_enabled = state['vision']
        except Exception:
            pass

    def load_model(self, model_path=None, device=None):
        """
        加载 YOLO 模型
        
        Args:
            model_path: 模型文件路径（如 yolov8n.pt, best.pt）
            device: 推理设备（cpu, cuda, 0）
            
        Returns:
            dict: 加载结果
        """
        if not YOLO_AVAILABLE:
            return {'success': False, 'error': 'ultralytics 未安装，请运行: pip install ultralytics'}

        if model_path:
            self._model_path = model_path
        if device:
            self._device = device

        try:
            with self._lock:
                self._model = YOLO(self._model_path)
                self._model_name = Path(self._model_path).stem

            # 预热模型
            if NUMPY_AVAILABLE:
                dummy = np.zeros((self._imgsz, self._imgsz, 3), dtype=np.uint8)
                self._model.predict(source=dummy, conf=self._confidence,
                                    iou=self._iou_threshold, device=self._device,
                                    verbose=False)

            return {
                'success': True,
                'model_name': self._model_name,
                'model_path': self._model_path,
                'device': self._device,
                'classes': self._model.model.names if hasattr(self._model, 'model') else {}
            }
        except Exception as e:
            return {'success': False, 'error': f'模型加载失败: {e}'}

    def detect(self, image, confidence=None, iou_threshold=None, classes_filter=None):
        """
        执行目标检测
        
        Args:
            image: 输入图像（numpy数组或文件路径）
            confidence: 置信度阈值
            iou_threshold: IOU阈值
            classes_filter: 类别过滤列表
            
        Returns:
            dict: 检测结果
        """
        if self._simulation_enabled:
            return self._simulate_detection(image)

        if not YOLO_AVAILABLE or self._model is None:
            if not YOLO_AVAILABLE:
                return {'success': False, 'error': 'ultralytics 未安装'}
            return {'success': False, 'error': '模型未加载，请先调用 load_model()'}

        conf = confidence if confidence is not None else self._confidence
        iou = iou_threshold if iou_threshold is not None else self._iou_threshold
        filter_cls = classes_filter if classes_filter is not None else self._classes_filter

        try:
            with self._lock:
                start_time = time.time()
                results = self._model.predict(
                    source=image,
                    conf=conf,
                    iou=iou,
                    device=self._device,
                    verbose=False
                )
                inference_time = (time.time() - start_time) * 1000.0

            if not results:
                return {
                    'success': True,
                    'detections': [],
                    'inference_time': inference_time,
                    'model_name': self._model_name
                }

            result = results[0]

            # 使用 supervision 处理检测结果
            if SUPERVISION_AVAILABLE:
                detections = sv.Detections.from_ultralytics(result)

                # 应用类别过滤
                if filter_cls:
                    class_names = self._model.model.names
                    filter_ids = [k for k, v in class_names.items() if v in filter_cls]
                    if filter_ids:
                        detections = detections[np.isin(detections.class_id, filter_ids)]

                # 应用 NMS
                detections = detections.with_nms(threshold=iou)

                return {
                    'success': True,
                    'detections': self._detections_to_dict(detections),
                    'inference_time': inference_time,
                    'model_name': self._model_name,
                    'sv_detections': detections
                }
            else:
                # 不使用 supervision 时的基本处理
                det_list = []
                if result.boxes is not None:
                    for box in result.boxes:
                        cls_id = int(box.cls[0])
                        conf_val = float(box.conf[0])
                        class_name = self._model.model.names.get(cls_id, str(cls_id))

                        if filter_cls and class_name not in filter_cls:
                            continue

                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        det_list.append({
                            'class_name': class_name,
                            'confidence': conf_val,
                            'class_id': cls_id,
                            'bbox': [float(x1), float(y1), float(x2), float(y2)]
                        })

                return {
                    'success': True,
                    'detections': det_list,
                    'inference_time': inference_time,
                    'model_name': self._model_name
                }

        except Exception as e:
            return {'success': False, 'error': f'检测失败: {e}'}

    def annotate_image(self, image, detections_data, annotators=None):
        """
        使用 supervision 标注图像
        
        Args:
            image: 原始图像
            detections_data: detect() 返回的结果
            annotators: 要使用的标注器列表，如 ['box', 'label', 'mask']
            
        Returns:
            numpy.ndarray: 标注后的图像
        """
        if not SUPERVISION_AVAILABLE or not CV2_AVAILABLE:
            if CV2_AVAILABLE:
                return image
            return None

        if not annotators:
            annotators = ['box', 'label']

        try:
            annotated = image.copy()

            if 'sv_detections' not in detections_data:
                return annotated

            sv_dets = detections_data['sv_detections']

            for ann_type in annotators:
                if ann_type == 'box' and self._box_annotator:
                    annotated = self._box_annotator.annotate(
                        scene=annotated, detections=sv_dets)
                elif ann_type == 'label' and self._label_annotator:
                    labels = []
                    if hasattr(sv_dets, 'class_id') and sv_dets.class_id is not None:
                        class_names = self._model.model.names if self._model else {}
                        for cid in sv_dets.class_id:
                            labels.append(class_names.get(int(cid), str(cid)))
                    annotated = self._label_annotator.annotate(
                        scene=annotated, detections=sv_dets, labels=labels)
                elif ann_type == 'mask' and self._mask_annotator and sv_dets.mask is not None:
                    annotated = self._mask_annotator.annotate(
                        scene=annotated, detections=sv_dets)
                elif ann_type == 'round_box' and self._round_box_annotator:
                    annotated = self._round_box_annotator.annotate(
                        scene=annotated, detections=sv_dets)
                elif ann_type == 'color' and self._color_annotator:
                    annotated = self._color_annotator.annotate(
                        scene=annotated, detections=sv_dets)

            return annotated

        except Exception:
            return image

    def add_zone(self, zone_name, polygon_points):
        """
        添加监控区域
        
        Args:
            zone_name: 区域名称
            polygon_points: 多边形顶点列表 [[x1,y1], [x2,y2], ...]
            
        Returns:
            dict: 操作结果
        """
        if not SUPERVISION_AVAILABLE:
            return {'success': False, 'error': 'supervision 未安装'}

        try:
            polygon = np.array(polygon_points, dtype=np.int32)
            zone = sv.PolygonZone(polygon=polygon)
            self._zones[zone_name] = zone
            self._zone_annotators[zone_name] = sv.PolygonZoneAnnotator(zone=zone)
            return {'success': True, 'zone_name': zone_name}
        except Exception as e:
            return {'success': False, 'error': f'添加区域失败: {e}'}

    def add_line_zone(self, line_name, start_point, end_point):
        """
        添加越线检测线
        
        Args:
            line_name: 线名称
            start_point: 起始点 [x, y]
            end_point: 结束点 [x, y]
            
        Returns:
            dict: 操作结果
        """
        if not SUPERVISION_AVAILABLE:
            return {'success': False, 'error': 'supervision 未安装'}

        try:
            start = sv.Point(x=start_point[0], y=start_point[1])
            end = sv.Point(x=end_point[0], y=end_point[1])
            line_zone = sv.LineZone(start=start, end=end)
            self._line_zones[line_name] = line_zone
            self._line_zone_annotators[line_name] = sv.LineZoneAnnotator(
                trigger=line_zone)
            return {'success': True, 'line_name': line_name}
        except Exception as e:
            return {'success': False, 'error': f'添加越线检测失败: {e}'}

    def get_zone_counts(self, detections_data):
        """
        获取各区域内的目标数量
        
        Args:
            detections_data: detect() 返回的结果
            
        Returns:
            dict: 各区域的目标数量
        """
        if not SUPERVISION_AVAILABLE or 'sv_detections' not in detections_data:
            return {}

        sv_dets = detections_data['sv_detections']
        counts = {}
        for name, zone in self._zones.items():
            zone.trigger(detections=sv_dets)
            counts[name] = zone.current_count
        return counts

    def get_line_crossings(self, detections_data):
        """
        获取各越线检测线的计数
        
        Args:
            detections_data: detect() 返回的结果
            
        Returns:
            dict: 各越线检测的进出计数
        """
        if not SUPERVISION_AVAILABLE or 'sv_detections' not in detections_data:
            return {}

        sv_dets = detections_data['sv_detections']
        crossings = {}
        for name, line_zone in self._line_zones.items():
            line_zone.trigger(detections=sv_dets)
            crossings[name] = {
                'in_count': line_zone.in_count,
                'out_count': line_zone.out_count
            }
        return crossings

    def train_model(self, dataset_path, model_type='yolov8n', epochs=100,
                    learning_rate=0.01, imgsz=640, batch_size=16,
                    augmentation=True, output_path=None, fine_tune_from=None,
                    callback=None):
        """
        训练或微调 YOLO 模型
        
        Args:
            dataset_path: 数据集路径（需包含 data.yaml）
            model_type: 基础模型类型（如 yolov8n, yolov8s, yolov8m）
            epochs: 训练轮数
            learning_rate: 初始学习率
            imgsz: 输入图像大小
            batch_size: 批次大小
            augmentation: 是否启用数据增强
            output_path: 模型输出路径
            fine_tune_from: 微调基础模型路径（如果指定则进行微调）
            callback: 训练进度回调函数
            
        Returns:
            dict: 训练结果
        """
        if not YOLO_AVAILABLE:
            return {'success': False, 'error': 'ultralytics 未安装，请运行: pip install ultralytics'}

        with self._training_lock:
            if self._training_active:
                return {'success': False, 'error': '已有训练任务正在运行'}

        data_yaml = os.path.join(dataset_path, 'data.yaml')
        if not os.path.exists(data_yaml):
            return {'success': False, 'error': f'数据集配置文件不存在: {data_yaml}'}

        if not output_path:
            output_path = self._model_output_path

        os.makedirs(output_path, exist_ok=True)

        # 启动训练线程
        thread = threading.Thread(
            target=self._run_training,
            args=(data_yaml, model_type, epochs, learning_rate, imgsz,
                  batch_size, augmentation, output_path, fine_tune_from, callback),
            daemon=True
        )
        thread.start()

        return {'success': True, 'message': '训练已启动'}

    def _run_training(self, data_yaml, model_type, epochs, learning_rate,
                      imgsz, batch_size, augmentation, output_path,
                      fine_tune_from, callback):
        """
        执行训练（在独立线程中运行）
        """
        with self._training_lock:
            self._training_active = True
            self._cancel_training = False
            self._training_progress = {
                'status': 'starting',
                'epoch': 0,
                'total_epochs': epochs,
                'loss': None,
                'map50': None,
                'model_type': model_type,
                'fine_tune': fine_tune_from is not None
            }

        start_time = time.time()

        try:
            # 加载基础模型
            if fine_tune_from:
                model = YOLO(fine_tune_from)
                self._training_progress['status'] = 'fine_tuning'
            else:
                model = YOLO(f'{model_type}.pt')
                self._training_progress['status'] = 'training'

            # 训练参数
            train_args = {
                'data': data_yaml,
                'epochs': epochs,
                'lr0': learning_rate,
                'imgsz': imgsz,
                'batch': batch_size,
                'project': output_path,
                'name': 'train',
                'exist_ok': True,
                'verbose': True,
                'patience': 50,
                'save_period': 10,
            }

            # 数据增强参数
            if augmentation:
                train_args.update({
                    'hsv_h': 0.015,
                    'hsv_s': 0.7,
                    'hsv_v': 0.4,
                    'degrees': 10.0,
                    'translate': 0.1,
                    'scale': 0.5,
                    'shear': 2.0,
                    'flipud': 0.0,
                    'fliplr': 0.5,
                    'mosaic': 1.0,
                    'mixup': 0.1,
                    'copy_paste': 0.1,
                })
            else:
                train_args.update({
                    'hsv_h': 0.0, 'hsv_s': 0.0, 'hsv_v': 0.0,
                    'degrees': 0.0, 'translate': 0.0, 'scale': 0.0,
                    'shear': 0.0, 'flipud': 0.0, 'fliplr': 0.0,
                    'mosaic': 0.0, 'mixup': 0.0, 'copy_paste': 0.0,
                })

            # 执行训练
            results = model.train(**train_args)

            elapsed = time.time() - start_time
            best_model = os.path.join(output_path, 'train', 'weights', 'best.pt')

            self._training_progress.update({
                'status': 'completed',
                'training_time': elapsed,
                'best_model_path': best_model if os.path.exists(best_model) else None,
            })

            if callback:
                callback(self._training_progress)

        except Exception as e:
            elapsed = time.time() - start_time
            self._training_progress.update({
                'status': 'failed',
                'error': str(e),
                'training_time': elapsed,
            })
            if callback:
                callback(self._training_progress)

        finally:
            with self._training_lock:
                self._training_active = False

    def cancel_training(self):
        """取消当前训练任务"""
        with self._training_lock:
            if self._training_active:
                self._cancel_training = True
                return {'success': True, 'message': '已请求取消训练'}
            return {'success': False, 'error': '没有正在运行的训练任务'}

    def get_training_status(self):
        """
        获取训练状态
        
        Returns:
            dict: 训练状态信息
        """
        with self._training_lock:
            return dict(self._training_progress)

    def fine_tune(self, dataset_path, base_model_path, epochs=50,
                  learning_rate=0.001, imgsz=640, batch_size=16,
                  output_path=None, callback=None):
        """
        微调已有模型
        
        Args:
            dataset_path: 数据集路径
            base_model_path: 基础模型路径（如 best.pt）
            epochs: 微调轮数（通常比训练少）
            learning_rate: 学习率（通常比训练小）
            imgsz: 图像大小
            batch_size: 批次大小
            output_path: 输出路径
            callback: 进度回调
            
        Returns:
            dict: 微调结果
        """
        if not os.path.exists(base_model_path):
            return {'success': False, 'error': f'基础模型不存在: {base_model_path}'}

        return self.train_model(
            dataset_path=dataset_path,
            model_type='custom',
            epochs=epochs,
            learning_rate=learning_rate,
            imgsz=imgsz,
            batch_size=batch_size,
            output_path=output_path,
            fine_tune_from=base_model_path,
            callback=callback
        )

    def validate_model(self, model_path=None, data_yaml=None):
        """
        验证模型性能
        
        Args:
            model_path: 模型路径（默认使用当前模型）
            data_yaml: 验证数据集配置
            
        Returns:
            dict: 验证结果
        """
        if not YOLO_AVAILABLE:
            return {'success': False, 'error': 'ultralytics 未安装'}

        try:
            if model_path:
                model = YOLO(model_path)
            elif self._model:
                model = self._model
            else:
                return {'success': False, 'error': '模型未加载'}

            val_args = {}
            if data_yaml:
                val_args['data'] = data_yaml

            results = model.val(**val_args)

            return {
                'success': True,
                'map50': float(results.box.map50) if hasattr(results, 'box') else None,
                'map50_95': float(results.box.map) if hasattr(results, 'box') else None,
                'precision': float(results.box.mp) if hasattr(results, 'box') else None,
                'recall': float(results.box.mr) if hasattr(results, 'box') else None,
            }
        except Exception as e:
            return {'success': False, 'error': f'验证失败: {e}'}

    def export_model(self, model_path=None, format='onnx', output_path=None):
        """
        导出模型到不同格式
        
        Args:
            model_path: 模型路径
            format: 导出格式（onnx, torchscript, tflite, etc.）
            output_path: 输出路径
            
        Returns:
            dict: 导出结果
        """
        if not YOLO_AVAILABLE:
            return {'success': False, 'error': 'ultralytics 未安装'}

        try:
            if model_path:
                model = YOLO(model_path)
            elif self._model:
                model = self._model
            else:
                return {'success': False, 'error': '模型未加载'}

            export_path = model.export(format=format)
            return {'success': True, 'export_path': str(export_path), 'format': format}
        except Exception as e:
            return {'success': False, 'error': f'导出失败: {e}'}

    def manage_dataset(self, dataset_path, action='info', **kwargs):
        """
        管理数据集（使用 supervision 数据集工具）
        
        Args:
            dataset_path: 数据集路径
            action: 操作类型（info, split, merge, convert）
            **kwargs: 额外参数
            
        Returns:
            dict: 操作结果
        """
        if not SUPERVISION_AVAILABLE:
            return {'success': False, 'error': 'supervision 未安装'}

        try:
            if action == 'info':
                return self._dataset_info(dataset_path)
            elif action == 'split':
                return self._dataset_split(dataset_path, **kwargs)
            elif action == 'convert':
                return self._dataset_convert(dataset_path, **kwargs)
            else:
                return {'success': False, 'error': f'未知操作: {action}'}
        except Exception as e:
            return {'success': False, 'error': f'数据集操作失败: {e}'}

    def _dataset_info(self, dataset_path):
        """获取数据集信息"""
        data_yaml = os.path.join(dataset_path, 'data.yaml')
        if not os.path.exists(data_yaml):
            return {'success': False, 'error': 'data.yaml 不存在'}

        images_dir = os.path.join(dataset_path, 'images')
        labels_dir = os.path.join(dataset_path, 'labels')

        info = {
            'success': True,
            'path': dataset_path,
            'images': {
                'train': len(os.listdir(os.path.join(images_dir, 'train'))) if os.path.exists(os.path.join(images_dir, 'train')) else 0,
                'val': len(os.listdir(os.path.join(images_dir, 'val'))) if os.path.exists(os.path.join(images_dir, 'val')) else 0,
            },
            'labels': {
                'train': len(os.listdir(os.path.join(labels_dir, 'train'))) if os.path.exists(os.path.join(labels_dir, 'train')) else 0,
                'val': len(os.listdir(os.path.join(labels_dir, 'val'))) if os.path.exists(os.path.join(labels_dir, 'val')) else 0,
            }
        }
        return info

    def _dataset_split(self, dataset_path, train_ratio=0.7, val_ratio=0.2, test_ratio=0.1):
        """分割数据集"""
        try:
            ds = sv.DetectionDataset.from_yolo(
                images_directory_path=os.path.join(dataset_path, 'images'),
                annotations_directory_path=os.path.join(dataset_path, 'labels'),
            )
            train_ds, val_ds, test_ds = sv.split_dataset(
                ds, ratios=[train_ratio, val_ratio, test_ratio]
            )
            return {
                'success': True,
                'train_size': len(train_ds),
                'val_size': len(val_ds),
                'test_size': len(test_ds),
            }
        except Exception as e:
            return {'success': False, 'error': str(e)}

    def _dataset_convert(self, dataset_path, target_format='coco', output_dir=None):
        """转换数据集格式"""
        try:
            ds = sv.DetectionDataset.from_yolo(
                images_directory_path=os.path.join(dataset_path, 'images'),
                annotations_directory_path=os.path.join(dataset_path, 'labels'),
            )
            if not output_dir:
                output_dir = os.path.join(dataset_path, f'converted_{target_format}')

            if target_format == 'coco':
                ds.as_coco(output_dir=output_dir)
            elif target_format == 'voc':
                ds.as_pascal_voc(output_dir=output_dir)

            return {'success': True, 'output_dir': output_dir, 'format': target_format}
        except Exception as e:
            return {'success': False, 'error': str(e)}

    def _detections_to_dict(self, sv_detections):
        """将 supervision Detections 转换为字典列表"""
        det_list = []
        class_names = self._model.model.names if self._model and hasattr(self._model, 'model') else {}

        for i in range(len(sv_detections)):
            det = {
                'class_id': int(sv_detections.class_id[i]) if sv_detections.class_id is not None else None,
                'class_name': class_names.get(int(sv_detections.class_id[i]), str(sv_detections.class_id[i])) if sv_detections.class_id is not None else None,
                'confidence': float(sv_detections.confidence[i]) if sv_detections.confidence is not None else None,
                'bbox': sv_detections.xyxy[i].tolist() if sv_detections.xyxy is not None else None,
                'tracker_id': int(sv_detections.tracker_id[i]) if sv_detections.tracker_id is not None else None,
            }
            det_list.append(det)

        return det_list

    def _simulate_detection(self, image):
        """仿真模式下的检测结果"""
        if not NUMPY_AVAILABLE:
            return {'success': False, 'error': 'numpy 未安装'}

        h, w = image.shape[:2] if image is not None and len(image.shape) == 3 else (480, 640)

        simulated_detections = [
            {
                'class_name': 'person',
                'confidence': 0.92,
                'class_id': 0,
                'bbox': [w * 0.1, h * 0.2, w * 0.4, h * 0.9]
            },
            {
                'class_name': 'car',
                'confidence': 0.85,
                'class_id': 2,
                'bbox': [w * 0.5, h * 0.3, w * 0.9, h * 0.8]
            },
            {
                'class_name': 'bottle',
                'confidence': 0.78,
                'class_id': 39,
                'bbox': [w * 0.6, h * 0.1, w * 0.75, h * 0.4]
            }
        ]

        return {
            'success': True,
            'detections': simulated_detections,
            'inference_time': 15.0,
            'model_name': 'simulation',
            'simulation': True
        }

    def set_simulation(self, enabled):
        """设置仿真模式"""
        self._simulation_enabled = enabled
        try:
            from agv_web_server.database_manager import DatabaseManager
            db = DatabaseManager()
            db.set_simulation_state('vision', enabled)
        except Exception:
            pass

    def get_simulation(self):
        """获取仿真模式状态"""
        return self._simulation_enabled

    def get_model_info(self):
        """获取当前模型信息"""
        info = {
            'model_loaded': self._model is not None,
            'model_path': self._model_path,
            'model_name': self._model_name,
            'confidence': self._confidence,
            'iou_threshold': self._iou_threshold,
            'device': self._device,
            'imgsz': self._imgsz,
            'classes_filter': self._classes_filter,
            'simulation_enabled': self._simulation_enabled,
            'supervision_available': SUPERVISION_AVAILABLE,
            'yolo_available': YOLO_AVAILABLE,
            'zones': list(self._zones.keys()),
            'line_zones': list(self._line_zones.keys()),
        }

        if self._model and hasattr(self._model, 'model'):
            info['classes'] = self._model.model.names

        return info

    def set_confidence(self, confidence):
        """设置置信度阈值"""
        if 0.0 <= confidence <= 1.0:
            self._confidence = confidence
            return {'success': True, 'confidence': confidence}
        return {'success': False, 'error': '置信度必须在 0.0 到 1.0 之间'}

    def set_iou_threshold(self, threshold):
        """设置IOU阈值"""
        if 0.0 <= threshold <= 1.0:
            self._iou_threshold = threshold
            return {'success': True, 'iou_threshold': threshold}
        return {'success': False, 'error': 'IOU阈值必须在 0.0 到 1.0 之间'}

    def set_device(self, device):
        """设置推理设备"""
        self._device = device
        return {'success': True, 'device': device}

    def set_classes_filter(self, classes):
        """设置类别过滤"""
        self._classes_filter = classes
        return {'success': True, 'classes_filter': classes}

    def get_available_models(self):
        """获取可用的 YOLO 模型列表"""
        models = [
            {'name': 'yolov8n', 'description': 'YOLOv8 Nano - 最小最快', 'size': '6.3MB'},
            {'name': 'yolov8s', 'description': 'YOLOv8 Small - 平衡', 'size': '22.5MB'},
            {'name': 'yolov8m', 'description': 'YOLOv8 Medium - 中等', 'size': '52.0MB'},
            {'name': 'yolov8l', 'description': 'YOLOv8 Large - 高精度', 'size': '87.0MB'},
            {'name': 'yolov8x', 'description': 'YOLOv8 Extra Large - 最高精度', 'size': '136.0MB'},
            {'name': 'yolov8n-seg', 'description': 'YOLOv8 Nano 分割', 'size': '6.7MB'},
            {'name': 'yolov8s-seg', 'description': 'YOLOv8 Small 分割', 'size': '23.7MB'},
            {'name': 'yolov8n-pose', 'description': 'YOLOv8 Nano 姿态', 'size': '6.5MB'},
        ]

        # 检查本地自定义模型
        if os.path.exists(self._model_output_path):
            for root, dirs, files in os.walk(self._model_output_path):
                for f in files:
                    if f.endswith('.pt'):
                        model_path = os.path.join(root, f)
                        models.append({
                            'name': f,
                            'description': f'自定义模型 - {f}',
                            'path': model_path
                        })

        return models
