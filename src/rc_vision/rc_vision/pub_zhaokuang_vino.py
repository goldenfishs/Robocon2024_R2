import argparse
import time
import cv2
import numpy as np
import random
from openvino.runtime import Core  # pip install openvino -i  https://pypi.tuna.tsinghua.edu.cn/simple
import onnxruntime as ort  # 使用onnxruntime推理用上，pip install onnxruntime，默认安装CPU
import pyrealsense2 as rs
import rclpy
import os
from rclpy.node import Node
from std_msgs.msg import Int64

class MyNode(Node):
    def __init__(self):
        super().__init__('zhaokuang')
        self.num_publisher = self.create_publisher(Int64, 'zhaokuang', 10)

    def publish_num(self, num):
        msg = Int64()
        if num is not None:
            msg.data = int(num)  # 将 num 转换为整数
        else:
            msg.data = 0  # 如果 num 是 None，设置默认值为 0
        self.num_publisher.publish(msg)


# 找框的15类用来给检测的原始结果赋值类别名
CLASSES = ['red_1', 'blue_2', 'redred_3', 'blueblue_4', 'redblue_5', 'bluered_6', 'redredred_7', 'blueblueblue_8',
           'redredblue_9', 'redbluered_10', 'redblueblue_11', 'bluebluered_12', 'blueredblue_13', 'blueredred_14',
           'empty_15']

# 用于确定五个框相对位置的原始列表
LEFT_12345_RIGHT_1 = []
# 已经赋值12345的字典
LEFT_12345_RIGHT_3 = []

# 红方的用于决策的字典
PRIORITY_DICT = {
                'red_1': 5, 'blue_2': 4, 'redred_3': 6, 'blueblue_4': 1, 'redblue_5': 2, 'bluered_6': 3,
                'redredred_7': 8, 'blueblueblue_8': 9, 'redredblue_9': 10, 'redbluered_10': 11,
                'redblueblue_11': 12, 'bluebluered_12': 13, 'blueredblue_13': 14, 'blueredred_14': 15, 'empty_15': 7
                }
# 蓝方的用于决策的字典
# PRIORITY_DICT = {
#     'red_1': 5, 'blue_2': 4, 'redred_3': 1, 'blueblue_4': 6, 'redblue_5': 3, 'bluered_6': 2,
#     'redredred_7': 8, 'blueblueblue_8': 9, 'redredblue_9': 10, 'redbluered_10': 11,
#     'redblueblue_11': 12, 'bluebluered_12': 13, 'blueredblue_13': 14, 'blueredred_14': 15,
#     'empty_15': 7
# }


class DepthCamera:
    """the tools for DepthCamera , include init and get frame and release."""
    def __init__(self):
        # 实例化realsense模块
        self.pipeline = rs.pipeline()
        # 创建config对象
        config = rs.config()
        config.enable_device('141222073956')
        # config.enable_device('815412071298')

        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()

        found_rgb = False
        for sensor in device.sensors:
            if sensor.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)

        # 声明RGB和深度视频流
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # 启动数据流
        self.pipeline.start(config)
        self.align_to_color = rs.align(rs.stream.color)  # 对齐rgb和深度图

    def get_frame(self, pickball_or_findbest):
        frames = self.pipeline.wait_for_frames()
        frames = self.align_to_color.process(frames)  # 使用process来实现宣告的align对齐功能 将上图获取的视频帧对齐
        # 将合成帧分开
        if pickball_or_findbest == 0:
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            # 转换成numpy数据
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            if not len(depth_image) or not len(color_image):
                return None, None
            else:
                return depth_image, color_image
        else:
            color_frame = frames.get_color_frame()
            # 转换成numpy数据
            color_image = np.asanyarray(color_frame.get_data())
            if not len(color_image):
                return None
            else:
                return color_image

    def release(self):
        self.pipeline.stop()


class OpenvinoInference(object):
    def __init__(self, onnx_path):
        self.onnx_path = onnx_path
        ie = Core()
        self.model_onnx = ie.read_model(model=self.onnx_path)
        self.compiled_model_onnx = ie.compile_model(model=self.model_onnx, device_name="CPU")
        self.output_layer_onnx = self.compiled_model_onnx.output(0)

    def predict(self, datas):
        predict_data = self.compiled_model_onnx([datas])[self.output_layer_onnx]
        return predict_data


class YOLOv8:
    """YOLOv8 object detection model class for handling inference and visualization."""

    def __init__(self, onnx_model, imgsz=(640, 640), infer_tool='openvino'):
        """
        Initialization.

        Args:
            onnx_model (str): Path to the ONNX model.
        """
        self.infer_tool = infer_tool
        if self.infer_tool == 'openvino':
            # 构建openvino推理引擎
            self.openvino = OpenvinoInference(onnx_model)
            self.ndtype = np.single
        else:
            # 构建onnxruntime推理引擎
            self.ort_session = ort.InferenceSession(onnx_model,
                                                    providers=['CUDAExecutionProvider', 'CPUExecutionProvider']
                                                    if ort.get_device() == 'GPU' else ['CPUExecutionProvider'])

            # Numpy dtype: support both FP32 and FP16 onnx model
            self.ndtype = np.half if self.ort_session.get_inputs()[0].type == 'tensor(float16)' else np.single

        self.classes = CLASSES  # 加载模型类别
        self.model_height, self.model_width = imgsz[0], imgsz[1]  # 图像resize大小
        self.color_palette = np.random.uniform(0, 255, size=(len(self.classes), 3))  # 为每个类别生成调色板

    def __call__(self, im0, conf_threshold=0.4, iou_threshold=0.45):
        """
        The whole pipeline: pre-process -> inference -> post-process.

        Args:
            im0 (Numpy.ndarray): original input image.
            conf_threshold (float): confidence threshold for filtering predictions.
            iou_threshold (float): iou threshold for NMS.

        Returns:
            boxes (List): list of bounding boxes.
        """
        # 前处理Pre-process
        t1 = time.time()
        im, ratio, (pad_w, pad_h) = self.preprocess(im0)
        print('预处理时间：{:.3f}s'.format(time.time() - t1))

        # 推理 inference
        t2 = time.time()
        if self.infer_tool == 'openvino':
            preds = self.openvino.predict(im)
        else:
            preds = self.ort_session.run(None, {self.ort_session.get_inputs()[0].name: im})[0]
        print('推理时间：{:.2f}s'.format(time.time() - t2))

        # 后处理Post-process
        t3 = time.time()
        boxes = self.postprocess(preds,
                                 im0=im0,
                                 ratio=ratio,
                                 pad_w=pad_w,
                                 pad_h=pad_h,
                                 conf_threshold=conf_threshold,
                                 iou_threshold=iou_threshold,
                                 )
        print('后处理时间：{:.3f}s'.format(time.time() - t3))

        return boxes

    # 前处理，包括：resize, pad, HWC to CHW，BGR to RGB，归一化，增加维度CHW -> BCHW
    def preprocess(self, img):
        """
        Pre-processes the input image.

        Args:
            img (Numpy.ndarray): image about to be processed.

        Returns:
            img_process (Numpy.ndarray): image preprocessed for inference.
            ratio (tuple): width, height ratios in letterbox.
            pad_w (float): width padding in letterbox.
            pad_h (float): height padding in letterbox.
        """
        # Resize and pad input image using letterbox() (Borrowed from Ultralytics)
        shape = img.shape[:2]  # original image shape
        new_shape = (self.model_height, self.model_width)
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        ratio = r, r
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        pad_w, pad_h = (new_shape[1] - new_unpad[0]) / 2, (new_shape[0] - new_unpad[1]) / 2  # wh padding
        if shape[::-1] != new_unpad:  # resize
            img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(pad_h - 0.1)), int(round(pad_h + 0.1))
        left, right = int(round(pad_w - 0.1)), int(round(pad_w + 0.1))
        img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114))  # 填充

        # Transforms: HWC to CHW -> BGR to RGB -> div(255) -> contiguous -> add axis(optional)
        img = np.ascontiguousarray(np.einsum('HWC->CHW', img)[::-1], dtype=self.ndtype) / 255.0
        img_process = img[None] if len(img.shape) == 3 else img
        return img_process, ratio, (pad_w, pad_h)

    # 后处理，包括：阈值过滤与NMS
    def postprocess(self, preds, im0, ratio, pad_w, pad_h, conf_threshold, iou_threshold):
        """
        Post-process the prediction.

        Args:
            preds (Numpy.ndarray): predictions come from ort.session.run().
            im0 (Numpy.ndarray): [h, w, c] original input image.
            ratio (tuple): width, height ratios in letterbox.
            pad_w (float): width padding in letterbox.
            pad_h (float): height padding in letterbox.
            conf_threshold (float): conf threshold.
            iou_threshold (float): iou threshold.

        Returns:
            boxes (List): list of bounding boxes.
        """
        x = preds  # outputs: predictions (1, 84, 8400)
        # Transpose the first output:
        # (Batch_size, xywh_conf_cls, Num_anchors) -> (Batch_size, Num_anchors, xywh_conf_cls)
        x = np.einsum('bcn->bnc', x)  # (1, 8400, 84)

        # Predictions filtering by conf-threshold
        x = x[np.amax(x[..., 4:], axis=-1) > conf_threshold]

        # Create a new matrix which merge these(box, score, cls) into one
        # For more details about `numpy.c_()`: https://numpy.org/doc/1.26/reference/generated/numpy.c_.html
        x = np.c_[x[..., :4], np.amax(x[..., 4:], axis=-1), np.argmax(x[..., 4:], axis=-1)]

        # NMS filtering
        # 经过NMS后的值, np.array([[x, y, w, h, conf, cls], ...]), shape=(-1, 4 + 1 + 1)
        x = x[cv2.dnn.NMSBoxes(x[:, :4], x[:, 4], conf_threshold, iou_threshold)]

        # 重新缩放边界框，为画图做准备
        if len(x) > 0:
            # Bounding boxes format change: cxcywh -> xyxy
            x[..., [0, 1]] -= x[..., [2, 3]] / 2
            x[..., [2, 3]] += x[..., [0, 1]]

            # Rescales bounding boxes from model shape(model_height, model_width) to the shape of original image
            x[..., :4] -= [pad_w, pad_h, pad_w, pad_h]
            x[..., :4] /= min(ratio)

            # Bounding boxes boundary clamp
            x[..., [0, 2]] = x[:, [0, 2]].clip(0, im0.shape[1])
            x[..., [1, 3]] = x[:, [1, 3]].clip(0, im0.shape[0])

            return x[..., :6]  # boxes
        else:
            return []

    # 绘框
    def draw(self, im, bboxes):
        """
        Draw on image.

        Args:
            im (np.ndarray): original image, shape [h, w, c].
            bboxes (numpy.ndarray): [n, 6], n is number of bboxes.

        Returns:
            None
        """
        # Draw rectangles
        for (*box, conf, cls_) in bboxes:
            # draw bbox rectangle
            cv2.rectangle(im, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])),
                          self.color_palette[int(cls_)], 2, cv2.LINE_AA)
            cv2.putText(im, f'{self.classes[int(cls_)]}: {conf:.3f}', (int(box[0]), int(box[1] - 9)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, self.color_palette[int(cls_)], 2, cv2.LINE_AA)


class FilterateAndFindBest:
    def __init__(self):
        self.classes = CLASSES

    # boxes require 5
    def filterate(self, bboxes):
        if len(bboxes) == 1:
            return bboxes
        else:
            return []

    # 此方法用来对字典进行排序，再将所得列表的键和值重新写到一个新字典里，且以赋值1, 2, 3, 4, 5
    def sort_list_by_value(self):
        LEFT_12345_RIGHT_1.sort(key=lambda x: x[1])
        count = 1
        for item in LEFT_12345_RIGHT_1:
            LEFT_12345_RIGHT_3.append((f'{item[0]}', count))
            count += 1

    # 1, 2, 3, 4, 5
    def left_12345_right(self, bboxes):
        # 遍历结果从左到右存入列表1
        for (*box, conf, cls_) in bboxes:
            cls_name = self.classes[int(cls_)]
            LEFT_12345_RIGHT_1.append((f'{cls_name}', int(box[0])))

        # 调用sort对列表从小到大排序并重新赋值1, 2, 3, 4, 5
        self.sort_list_by_value()

    # 遍历推理结果，从五个结果中获取最好的
    def findbest_and_return_thebest(self, bboxes):
        the_best_cls_name = None
        the_best_bbox = None
        for (*box, conf, cls_) in bboxes:
            cls_name = self.classes[int(cls_)]
            if the_best_cls_name is None or PRIORITY_DICT[cls_name] < PRIORITY_DICT[the_best_cls_name]:
                the_best_cls_name = cls_name
                the_best_bbox = box
            else:
                pass
        return the_best_cls_name, the_best_bbox

    # 将最好的结果在图中标记出
    def mark_the_best_and_visualize(self, im, bbox, vis=False):
        if bbox is not None:
            mark_x = int((int(bbox[0]) + int(bbox[2])) / 2)
            mark_y = int((int(bbox[1]) + int(bbox[3])) / 2)
            cv2.circle(im, (mark_x, mark_y), radius=5, color=(0, 255, 0), thickness=-1)
        else:
            pass

        # Show image
        if vis:
            cv2.imshow('demo', im)

    # 获取相对位置
    def get_relative_position(self, the_best_cls_name):
        if len(LEFT_12345_RIGHT_3) != 0:
            for item in LEFT_12345_RIGHT_3:
                if item[0] == the_best_cls_name:
                    return item[0], item[1]
                else:
                    pass
        else:
            return None, None


if __name__ == '__main__':
    rclpy.init()
    global node
    node = MyNode()
    # Create an argument parser to handle command-line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', type=str, default='/home/ubuntu/r2_algorithm/src/rc_vision/rc_vision/weights/weights2/best(1).onnx', help='Path to ONNX model')
    parser.add_argument('--imgsz', type=tuple, default=(640, 640), help='Image input size')
    parser.add_argument('--conf', type=float, default=0.4, help='Confidence threshold')
    parser.add_argument('--iou', type=float, default=0.45, help='NMS IoU threshold')
    parser.add_argument('--infer_tool', type=str, default='openvino', choices=("openvino", "onnxruntime"),
                        help='选择推理引擎')
    args = parser.parse_args()

    # Build model 加载模型
    model = YOLOv8(args.model, args.imgsz, args.infer_tool)

    # Setup depthcamera 实例化相机模块
    depthcamera = DepthCamera()

    # Build filterate_and_findbest 实例化决策模块
    filterateandfindbest = FilterateAndFindBest()
    try:
        while True:
            # Getimage 获取RGB图像
            ColorImage = depthcamera.get_frame(pickball_or_findbest=1)

            # flip 翻转
            ColorImage = cv2.flip(ColorImage, -1)

            # Inference 推理
            boxes = model(ColorImage, conf_threshold=args.conf, iou_threshold=args.iou)

            # Draw 根据推理的结果绘框
            model.draw(ColorImage, boxes)

            # Filterate 要求每张图像中检测到五个结果才进行决策
            boxes_1 = filterateandfindbest.filterate(boxes)

            # 1, 2, 3, 4, 5 将获取的五个结果从左到右排列并分别以键值对形式赋值1, 2, 3, 4, 5
            filterateandfindbest.left_12345_right(boxes_1)
            # Find best 从五个结果中获得最好的一个 
            # 检查是否所有框的级别都相同
            # 确保 box 是正确的类型
            if len(set(box.level for box in boxes if hasattr(box, 'level'))) == 1:
                # 如果是，随机选择一个
                best_box = random.choice(boxes)
            else:
                 # 否则，按照原来的逻辑找出最佳的一个
                best_cls_name, best_box = filterateandfindbest.findbest_and_return_thebest(boxes_1)

            # Mark the best and visualize 在图像中将最好的那个用点标记并可视化
            filterateandfindbest.mark_the_best_and_visualize(ColorImage, best_box, vis=True)

            # Comparison 将获得的最佳选择的类别名与得到的五个结果相对位置的类别名进行比对并返回最佳选择的相对位置
            best_relative_position_cls_name, best_relative_position = filterateandfindbest.get_relative_position(best_cls_name)
            # 已经将 best_relative_position 转换为 Int64
            # print(best_relative_position_cls_name)
            # print(best_relative_position)
            node.publish_num(best_relative_position)
            node.get_logger().info("num: %s" % best_relative_position)

            # clear the LEFT_12345_RIGHT 清空用于确定位置的列表准备处理下一帧
            LEFT_12345_RIGHT_1.clear()
            LEFT_12345_RIGHT_3.clear()

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cv2.destroyAllWindows()
        depthcamera.release()
    rclpy.spin(node)