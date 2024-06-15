#!/usr/bin/python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

import serial
import struct
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64



class MyNode(Node):
    def __init__(self):
        super().__init__('zhaokuang')
        self.num_publisher = self.create_publisher(Int64, 'zhaokuang', 10)

    def publish_num(self, num):
        msg = Int64()
        msg.data = num
        self.num_publisher.publish(msg)
        self.get_logger().info("num: {}".format(num))
  
# 接收导航的信息，确定红蓝方       
# 定义优先级字典
PRIORITY_DICT = {
    'red_1': 5, 'blue_2': 4, 'redred_3': 6, 'blueblue_4': 1, 'redblue_5': 2, 'bluered_6': 3, 
    'redredred_7': 8, 'blueblueblue_8': 8, 'redredblue_9': 8, 'redbluered_10': 8, 
    'redblueblue_11': 8,  'blueredblue_13': 8, 'blueredred_14': 8, 'empty_15': 7
}

# # 蓝方
# PRIORITY_DICT = {
#     'Red_1': 5, 'Blue_2': 4, 'RedRed_3': 1, 'BlueBlue_4': 6, 'RedBlue_5': 3, 'BlueRed_6': 2, 
#     'RedRedRed_7': 8, 'BlueBlueBlue_8': 9, 'RedRedBlue_9': 10, 'RedBlueRed_10': 11, 
#     'RedBlueBlue_11': 12, 'BlueBlueRed_12': 13, 'BlueRedBlue_13': 14, 'BlueRedRed_14': 15, 
#     'Empty_15': 7
# }
# 加载 YOLOv8 模型
model_path = os.path.expanduser("/home/ubuntu/r2_algorithm/src/rc_vision/rc_vision/weights/weights-1700-kuang-15zhong/best.pt")
model = YOLO(model_path)

#创建滤波器
spatial = rs.spatial_filter()
temporal = rs.temporal_filter()
hole_filling = rs.hole_filling_filter()
disparity = rs.disparity_transform(True)
threshold = rs.threshold_filter(0.5,5) # 设置最小和最大深度值

# 配置深度和颜色流
# 创建一个管道
pipeline = rs.pipeline()
# 创建一个配置
config = rs.config()

# config.enable_device('814412072268')
# config.enable_device('141222073956')
config.enable_device('815412071298')


pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
# 启动深度流
config.enable_stream(rs.stream.depth, 640,480,rs.format.z16,30)
config.enable_stream(rs.stream.color, 640,480,rs.format.bgr8,30)
profile = pipeline.start(config)

# 获取深度帧并应用滤波器
frames = pipeline.wait_for_frames()
depth_frame = frames.get_depth_frame()
depth_frame = spatial.process(depth_frame)
depth_frame = temporal.process(depth_frame)
depth_frame = hole_filling.process(depth_frame)
depth_frame = disparity.process(depth_frame)
depth_frame = threshold.process(depth_frame)

# 获取像素深度
def get_pixel_depth(depth_frame, x, y):
    return depth_frame.get_distance(x, y) 

# 在图像上画一个点
def draw_point(image, point, color=(0, 255, 0), radius=5):
    cv2.circle(image, point, radius, color, -1)  # 画一个实心圆


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
    """处理检测结果"""
    highest_priority_depth = None
    highest_priority_cls_name, highest_priority_box = get_highest_priority_box(results)
    if highest_priority_cls_name is not None:
        x, y, w, h = map(int, highest_priority_box[:4])          # 获取最高优先级的检测框的坐标
        bottom_center_y = int(y + h / 2)                  # 获取中心的y坐标
        bottom_center_x = int(x + w / 2)                  # 获取中心的x坐标
        highest_priority_depth = get_pixel_depth(depth_frame, x, bottom_center_y)  # 获取底部中心的深度值
        draw_point(annotated_frame, (x, bottom_center_y))        # 在图像上画一个点
        # 在图像上写入name和深度值
        cv2.putText(annotated_frame, f"{highest_priority_cls_name}: {highest_priority_depth:.2f}m", (x, bottom_center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)   
        if 0 < bottom_center_x < 300:
            node.publish_num(1)
        elif 300 < bottom_center_x < 380:
            node.publish_num(2)
        elif 380 < bottom_center_x < 640:
            node.publish_num(3)
    return highest_priority_depth
    

# 处理一帧图像
def process_frame(color_frame, depth_frame):
    """处理一帧图像，进行目标检测并返回检测结果"""
    depth_image = np.asanyarray(depth_frame.get_data()) # 将深度帧转换为numpy数组
    color_image = np.asanyarray(color_frame.get_data()) # 将颜色帧转换为numpy数组
    results = model.predict(source=color_image)         # 进行目标检测
    annotated_frame = results[0].plot()                 # 绘制检测结果
    process_results(results, depth_frame, annotated_frame)  # 处理检测结果
    
    return annotated_frame                           # 返回带有检测结果的图像
def main():
    """主函数，开始流，处理帧，然后停止流"""
    rclpy.init()
    global node
    node = MyNode()
    try:
        while True:
            frames = pipeline.wait_for_frames()             # 等待一组帧：深度和颜色
            depth_frame = frames.get_depth_frame()          # 获取深度帧
            color_frame = frames.get_color_frame()          # 获取颜色帧
            if not color_frame or not depth_frame:          # 如果没有颜色帧或深度帧，则跳过
                continue
            annotated_frame = process_frame(color_frame, depth_frame)   # 处理一帧图像
            flipped_frame = cv2.flip(annotated_frame, -1)
            cv2.imshow('img', flipped_frame)              # 显示图像              # 显示图像
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        pipeline.stop()                                     # 停止流
        cv2.destroyAllWindows()
    rclpy.spin(node)
if __name__ == "__main__":
    main()
