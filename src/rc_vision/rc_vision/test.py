from collections import defaultdict
from sensor_msgs.msg import MultiDOFJointState
from geometry_msgs.msg import Point
import cv2
import numpy as np
import os
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


from ultralytics import YOLO

redball = 0.0
blueball = 1.0
purpleball = 2.0


class ModelSwitcher(Node):
    def __init__(self):
        super().__init__('model_switcher')
        self.pub_quqiu = self.create_publisher(Point, 'quqiu', 10)
    
    def pub_quqiu(self, x, y, min_dist):
        point_msg = Point()
        point_msg.data = [float(x), float(y), float(min_dist)]
        self.pub_quqiu.publish(point_msg)

def setup_camera():
    pipeline = rs.pipeline()
    config = rs.config()
    # config.enable_device('141222073956')
    # config.enable_device('815412071298')                                                 
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)
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

def track_objects(model, color_frame, depth_frame, track_history,model_switcher):
    results = model.track(color_frame, persist=True, conf=0.60)
    boxes, track_ids, classes = get_tracked_object_data(results)
    found_target = False  # 添加一个标志变量
    annotated_frame = results[0].plot() if results else None  # 如果没有结果，annotated_frame为None
    if boxes is not None:
        for box, track_id, cls in zip(boxes, track_ids, classes):
            if cls != blueball and cls != redball:
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
            found_target = True  # 如果找到了目标对象，就将标志变量设置为True
            break
    if not found_target:
        # 创建一个Point消息并设置其值为0
        point_msg = Point()
        point_msg.x = 0.0
        point_msg.y = 250.0
        point_msg.z = 0.0
        # 发布消息
        model_switcher.pub_quqiu.publish(point_msg)
        model_switcher.get_logger().info("quqiu: %s" % [0.0, 250.0, 0.0])
    return annotated_frame

def main():
    rclpy.init()
    model = YOLO('/home/ubuntu/r2_algorithm/src/rc_vision/rc_vision/weights/weights2/best.pt')
    ov_model = YOLO("/home/ubuntu/r2_algorithm/src/rc_vision/rc_vision/weights/weights2/best_openvino_model/")
    track_history = defaultdict(lambda: [])
    model_switcher = ModelSwitcher()
    pipeline = setup_camera()
    try:
        while True:
            color_frame, depth_frame = get_frame(pipeline)
            if color_frame is not None and depth_frame is not None:
                annotated_frame = track_objects(ov_model, color_frame, depth_frame, track_history,model_switcher)
                # if annotated_frame is not None:
                    # cv2.imshow("YOLOv8 Tracking", annotated_frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

    model_switcher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()