from collections import defaultdict
from sensor_msgs.msg import MultiDOFJointState
from geometry_msgs.msg import Point
import cv2
import numpy as np
import os
import time
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from std_msgs.msg import Int64
from std_msgs.msg import String
import  threading
from ultralytics import YOLO
import subprocess
redball = 0.0
blueball = 1.0
purpleball = 2.0


PRIORITY_DICT = {
                'red_1': 5, 'blue_2': 4, 'redred_3': 6, 'blueblue_4': 1, 'redblue_5': 2, 'bluered_6': 3, 
                'redredred_7': 8, 'blueblueblue_8': 9, 'redredblue_9': 10, 'redbluered_10': 11, 
                'redblueblue_11': 12, 'bluebluered_12': 13, 'blueredblue_13': 14, 'blueredred_14': 15, 'empty_15': 7
                }
 #  # # 蓝方
            # PRIORITY_DICT = {
            #     'red_1': 5, 'blue_2': 4, 'redred_3': 1, 'blueblue_4': 6, 'redblue_5': 3, 'bluered_6': 2, 
            #     'redredred_7': 8, 'blueblueblue_8': 9, 'redredblue_9': 10, 'redbluered_10': 11, 
            #     'redblueblue_11': 12, 'bluebluered_12': 13, 'blueredblue_13': 14, 'blueredred_14': 15, 
            #     'mpty_15': 7
            # }
       
def setup_camera():
    pipeline = rs.pipeline()
    config = rs.config()
    # config.enable_device('141222073956')
    config.enable_device('815412071298')
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    pipeline.start(config)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    return pipeline
            # 对深度帧应用滤波器
def apply_filters(frame):
    filters = [rs.hole_filling_filter(), rs.temporal_filter(), rs.spatial_filter()]
    for filter in filters:
        frame = filter.process(frame)
    return cv2.medianBlur(np.asanyarray(frame.get_data()).astype(np.uint8), 5)
            
                # 获取一帧
def get_frame(pipeline):
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()
    if not color_frame or not depth_frame:
        return None, None
    return np.asanyarray(color_frame.get_data()), apply_filters(depth_frame)

                # 获取跟踪对象的数据
def get_tracked_object_data(results):
    if results[0].boxes.id is not None:
        return results[0].boxes.xywh.cpu(), results[0].boxes.id.int().cpu().tolist(), results[0].boxes.cls.cpu().tolist()
    return None, None, None
            
def track_objects(model, color_frame, depth_frame, track_history):
    results = model.track(color_frame, persist=True, conf=0.60)
    boxes, track_ids, classes = get_tracked_object_data(results)
    found_target = False  # 添加一个标志变量
    annotated_frame = results[0].plot() if results else None  # 如果没有结果，annotated_frame为None
    if boxes is not None:
        for box, track_id, cls in zip(boxes, track_ids, classes):
            if cls != redball:
                continue
            x, y, w, h = box
            track = track_history[track_id]
            track.append((float(x), float(y)))
            if len(track) > 30:
                track.pop(0)
            points = np.hstack(track).astype(np.int32).reshape((-1, 1, 2))
            cv2.polylines(annotated_frame, [points], isClosed=False, color=(230, 230, 230), thickness=10)
                            # print(f"追踪目标的中心距是： {(float(x), float(y))} with depth {depth_frame[int(y), int(x)]}")
            x = (float(x)-320)/3.2
            y = (float(y)-240)*5.0/48.0
            print(f"追踪目标的中心距是： {x} {y}")
            min_dist = np.float32((depth_frame[int(y), int(x)])/100.0)
            print(f"追踪目标的深度是： {min_dist}")
            # 创建一个Point消息并设置其值
            point_msg = Point()
            point_msg.x = x
            point_msg.y = y
            point_msg.z = float(min_dist)
            
            model_switcher.pub_quqiu.publish(point_msg)
            model_switcher.get_logger().info("quqiu: %s" % [float(x), float(y), min_dist])
            model_switcher.found_target = True  # 如果找到了目标对象，就将标志变量设置为True
    if model_switcher.found_target == False:
        # 创建一个Point消息并设置其值为0
        point_msg = Point()
        point_msg.x = 0.0
        point_msg.y = 0.0
        point_msg.z = 0.0
        # 发布消息
        model_switcher.pub_quqiu.publish(point_msg)
        model_switcher.get_logger().info("quqiu: %s" % [0.0, 0.0, 0.0])



model1 = "/home/ubuntu/r2_algorithm/src/rc_vision/rc_vision/weights/weights2/best.pt"
model2 = "/home/ubuntu/r2_algorithm/src/rc_vision/rc_vision/weights/weights-1700-kuang-15zhong/best.pt"  
class ModelRunner(Node):
    def __init__(self):
        super().__init__('model_runner')
        self.sub_move_mode = self.create_subscription(Int64,'robot_mode',self.listener_callback,10)
        self.stop_flag = False
        self.thread = None
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%d"' % msg.data)
        # 当接收到2时，开始运行模型
        if msg.data == 2:
            self.start(model1)
            self.stop(model2)
        # 当接收到3时，停止运行模型
        elif msg.data == 3:
            self.stop(model1)
            self.start(model2)


class ModelSwitcher(Node):
    def __init__(self):
        super().__init__('model_switcher')
        self.pub_quqiu = self.create_publisher(Point, 'quqiu', 10)
        self.sub_move_mode = self.create_subscription(Int64,'robot_mode',self.move_mode_callback,10)
        self.pub_zhaokuang = self.create_publisher(Int64, 'zhaokuang', 10)
        self.found_target = False
        self.current_program = None  # 新增属性来保存当前正在运行的程序
    def move_mode_callback(self, msg):
        model = None
        if not hasattr(self, 'initialized'):
            self.initialized = True
        if msg.data == 2:
            print(2)
            self.model = "/home/ubuntu/r2_algorithm/src/rc_vision/rc_vision/weights/weights2/best.pt"
            model1 = YOLO(self.model)
            track_history = defaultdict(lambda: [])
            main(model1, track_history)
        
        elif msg.data == 3:
            print(3)
            self.model = "/home/ubuntu/r2_algorithm/src/rc_vision/rc_vision/weights/weights-1700-kuang-15zhong/best.pt"
            model2 = YOLO(self.model)
            def setup_camera():
                pipeline = rs.pipeline()
                config = rs.config()
                # config.enable_device('141222073956')
                config.enable_device('815412071298')
                pipeline_wrapper = rs.pipeline_wrapper(pipeline)
                pipeline_profile = config.resolve(pipeline_wrapper)
                device = pipeline_profile.get_device()
                # pipeline.start(config)
                config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
                config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
                return pipeline
            # 对深度帧应用滤波器
            def apply_filters(frame):
                filters = [rs.hole_filling_filter(), rs.temporal_filter(), rs.spatial_filter()]
                for filter in filters:
                    frame = filter.process(frame)
                return cv2.medianBlur(np.asanyarray(frame.get_data()).astype(np.uint8), 5)

            def get_pixel_depth(depth_frame, x, y):
                return depth_frame.get_distance(x, y) 

            # 在图像上画一个点
            def draw_point(image, point, color=(0, 255, 0), radius=5):
                cv2.circle(image, point, radius, color, -1)  # 画一个实心圆

            # 在图像上画一条线，连接开始点和图像的正中心
            def draw_line_to_center(annotated_frame, start_point):
            # 获取图像的中心点坐标
                center_x, center_y = annotated_frame.shape[1] // 2, annotated_frame.shape[0] // 2
                # 在图像上画一条线，连接开始点和图像的正中心
                cv2.line(annotated_frame, start_point, (center_x, center_y), (0, 255, 0), 2)

            # 获取框的中心坐标
            def get_center_coordinates(box):
                x, y, w, h = map(int, box[:4])
                return x + w // 2, y + h // 2

            # 获取底部中心的y坐标
            def get_bottom_center_y(box):
                _, y, _, h = map(int, box[:4])
                return y + h // 2 

            # 获取优先级最高的检测框
            def get_highest_priority_box(results):
                highest_priority_cls_name = None # 最高优先级的类名
                highest_priority_box = None      # 最高优先级的检测框
                for r in results:                # 遍历检测结果
                    if r.boxes.xywh.shape[0] > 0:   # 如果检测结果的数量大于0
                        for i, box in enumerate(r.boxes.xywh):  # 遍历检测框
                            cls_index = r.boxes.cls[i].item()   # 获取类别索引
                            cls_name = r.names[cls_index]       # 获取类别名称
                            if highest_priority_cls_name is None or PRIORITY_DICT[cls_name] < PRIORITY_DICT[highest_priority_cls_name]: # 如果当前类别的优先级高于最高优先级的类别
                                highest_priority_cls_name = cls_name    # 更新最高优先级的类别名称
                                highest_priority_box = box              # 更新最高优先级的检测框
                return highest_priority_cls_name, highest_priority_box  # 返回最高优先级的类别名称和检测框


            # 处理检测结果
            def process_results(results, depth_frame, annotated_frame):
            #"""处理检测结果"""
                msg = Int64()
                highest_priority_depth = None
                highest_priority_cls_name, highest_priority_box = get_highest_priority_box(results)
                if highest_priority_cls_name is not None:
                    x, y, w, h = map(int, highest_priority_box[:4])          # 获取最高优先级的检测框的坐标
                    bottom_center_y = int(y + h / 2)                  # 获取中心的y坐标
                    bottom_center_x = int(x + w / 2)                  # 获取中心的x坐标
                    highest_priority_depth = get_pixel_depth(depth_frame, x, bottom_center_y)  # 获取底部中心的深度值
                    highest_priority_angle = (x - 320) * 0.2                 # 获取底部的角度值
                    draw_point(annotated_frame, (x, bottom_center_y))        # 在图像上画一个点
                    # 在图像上写入name和深度值
                    cv2.putText(annotated_frame, f"{highest_priority_cls_name}: {highest_priority_depth:.2f}m", (x, bottom_center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)   
                    
                    if 0 < bottom_center_x < 300:
                        model_switcher.pub_zhaokuang.publish(msg)
                        self.get_logger().info('zhaokuang: %d' % 1)
                    elif 300 < bottom_center_x < 380:
                        model_switcher.pub_zhaokuang.publish(msg)
                        self.get_logger().info('zhaokuang: %d' % 2)
                    elif 380 < bottom_center_x < 640:
                        model_switcher.pub_zhaokuang.publish(msg)
                        self.get_logger().info('zhaokuang: %d' % 3)
                        # 调用划线函数
                    draw_line_to_center(annotated_frame, (x, y))
                return highest_priority_depth
                

            # 处理一帧图像
            def process_frame(color_frame, depth_frame,model):
                # """处理一帧图像，进行目标检测并返回检测结果"""
                model = YOLO('/home/ubuntu/r2_algorithm/src/rc_vision/rc_vision/weights/weights-1700-kuang-15zhong/best.pt')
                depth_image = np.asanyarray(depth_frame.get_data()) # 将深度帧转换为numpy数组
                color_image = np.asanyarray(color_frame.get_data()) # 将颜色帧转换为numpy数组
                results = model.predict(source=color_image)         # 进行目标检测
                annotated_frame = results[0].plot()                 # 绘制检测结果
                process_results(results, depth_frame, annotated_frame)  # 处理检测结果

                return annotated_frame                           # 返回带有检测结果的图像                               # 关闭所有窗口
            def main():
                if not rclpy.ok():
                    rclpy.init()
                model_switcher = ModelSwitcher() 
                pipeline = setup_camera()
                pipeline.start()
                try:
                    while True:
                        frames = pipeline.wait_for_frames()  # 等待一组帧：深度和颜色
                        depth_frame = frames.get_depth_frame()  # 获取深度帧
                        color_frame = frames.get_color_frame()  # 获取颜色帧
                        if not color_frame or not depth_frame:  # 如果没有颜色帧或深度帧，则跳过
                            continue
                        annotated_frame = process_frame(color_frame, depth_frame, model)  # 处理一帧图像
                        flipped_frame = cv2.flip(annotated_frame, -1)
                        cv2.imshow('img', flipped_frame)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            break
                finally:
                    pipeline.stop()  # 停止流
                    cv2.destroyAllWindows()  # 关闭所有窗口
                model_switcher.destroy_node()
                rclpy.spin(model_switcher)
                rclpy.shutdown()
            main()
    def main(self, model, track_history):
        model = YOLO('/home/ubuntu/r2_algorithm/src/rc_vision/rc_vision/weights/weights2/best.pt')
        track_history = defaultdict(lambda: [])
        pipeline = setup_camera()
        try:
            while True:
                color_frame, depth_frame = get_frame(pipeline)
                if color_frame is not None and depth_frame is not None:
                    annotated_frame = track_objects(model, color_frame, depth_frame, track_history)
                if annotated_frame is not None and annotated_frame.size > 0:
                    cv2.imshow("YOLOv8 Tracking", annotated_frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                pass
        finally:
            pipeline.stop()
            cv2.destroyAllWindows()
        
if __name__ == '__main__':
    rclpy.init()
    model_switcher = ModelSwitcher()
    rclpy.spin(model_switcher)
    rclpy.shutdown()
    






