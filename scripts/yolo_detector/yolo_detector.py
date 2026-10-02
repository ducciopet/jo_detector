#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import torch
import os
import time
from std_msgs.msg import Float64
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import Detection2D
from cv_bridge import CvBridge
from utils.tool import handle_preds
from module.detector import Detector


path_curr = os.path.dirname(__file__)
device = "cuda" if torch.cuda.is_available() else "cpu"

class yolo_detector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        print("[onboardDetector]: yolo detector init...")

        self.image_topic = str(self.declare_parameter('image_topic', '').value)
        self.detected_image_topic = str(self.declare_parameter('detected_image_topic', '').value)
        self.detected_bboxes_topic = str(self.declare_parameter('detected_bboxes_topic', '').value)
        self.inference_time_topic = str(self.declare_parameter('inference_time_topic', '').value)
        self.weights_path = str(self.declare_parameter('weights_path', '').value)
        self.class_names_path = str(self.declare_parameter('class_names_path', '').value)
        self.target_classes = list(self.declare_parameter('target_classes', ['person']).value)
        self.timer_period_sec = float(self.declare_parameter('timer_period_sec', 0.033).value)
        self.inference_size = int(self.declare_parameter('inference_size', 352).value)
        self.queue_size = int(self.declare_parameter('queue_size', 10).value)
        self.score_threshold = float(self.declare_parameter('score_threshold', 0.65).value)

        missing = []
        if not self.image_topic:
            missing.append('image_topic')
        if not self.detected_image_topic:
            missing.append('detected_image_topic')
        if not self.detected_bboxes_topic:
            missing.append('detected_bboxes_topic')
        if not self.inference_time_topic:
            missing.append('inference_time_topic')
        if not self.weights_path:
            missing.append('weights_path')
        if not self.class_names_path:
            missing.append('class_names_path')
        if not self.target_classes:
            missing.append('target_classes')
        if missing:
            self.get_logger().error('Missing required parameters in YAML: ' + ', '.join(missing))
            raise RuntimeError('YOLO parameter initialization failed')

        self.weights_path = os.path.join(path_curr, self.weights_path)
        self.class_names_path = os.path.join(path_curr, self.class_names_path)

        self.img_received = False
        self.img_detected = False

        # init and load
        self.model = Detector(80, True).to(device)
        self.model.load_state_dict(torch.load(self.weights_path, map_location=device))
        self.model.eval()

        # subscriber. rclpy has no image_transport transport-hint helper, so
        # subscribe directly to the mangled "<base_topic>/compressed" topic
        # (published automatically by any image_transport::Publisher-based
        # driver, as long as compressed_image_transport is installed — see
        # docker/Dockerfile).
        self.br = CvBridge()
        self.img_sub = self.create_subscription(
            CompressedImage, self.image_topic + '/compressed', self.image_callback, self.queue_size)

        # publisher
        self.img_pub = self.create_publisher(Image, self.detected_image_topic, self.queue_size)
        self.bbox_pub = self.create_publisher(Detection2DArray, self.detected_bboxes_topic, self.queue_size)
        self.time_pub = self.create_publisher(Float64, self.inference_time_topic, 1)

        # timers (period in seconds)
        self.create_timer(self.timer_period_sec, self.detect_callback)
        self.create_timer(self.timer_period_sec, self.vis_callback)
        self.create_timer(self.timer_period_sec, self.bbox_callback)
    
    def image_callback(self, msg):
        self.img = self.br.compressed_imgmsg_to_cv2(msg, "bgr8")
        self.img_received = True

    def detect_callback(self):
        start_time = time.time()
        if self.img_received:
            output = self.inference(self.img)
            self.detected_img, self.detected_bboxes = self.postprocess(self.img, output)
            self.img_detected = True
        elapsed = time.time() - start_time
        msg = Float64()
        msg.data = float(elapsed)
        self.time_pub.publish(msg)
        

    def vis_callback(self):
        if self.img_detected:
            img_msg = self.br.cv2_to_imgmsg(self.detected_img, "bgr8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            self.img_pub.publish(img_msg)

    def bbox_callback(self):
        if self.img_detected:
            bboxes_msg = Detection2DArray()
            for detected_box in self.detected_bboxes:
                if detected_box[4] in self.target_classes:
                    bbox_msg = Detection2D()
                    # detected_box format: [x1, y1, x2, y2, class]
                    x1 = float(detected_box[0])
                    y1 = float(detected_box[1])
                    x2 = float(detected_box[2])
                    y2 = float(detected_box[3])
                    # Store top-left corner in hypothesis pose and size in bbox
                    from vision_msgs.msg import ObjectHypothesisWithPose
                    hyp = ObjectHypothesisWithPose()
                    hyp.pose.pose.position.x = x1  # top-left x
                    hyp.pose.pose.position.y = y1  # top-left y
                    bbox_msg.results.append(hyp)
                    bbox_msg.bbox.size_x = float(abs(x2 - x1))  # width
                    bbox_msg.bbox.size_y = float(abs(y2 - y1))  # height
                    bboxes_msg.detections.append(bbox_msg)
            bboxes_msg.header.stamp = self.get_clock().now().to_msg()
            self.bbox_pub.publish(bboxes_msg)

    def inference(self, ori_img):
        # image pre-processing
        res_img = cv2.resize(ori_img, (self.inference_size, self.inference_size), interpolation = cv2.INTER_LINEAR)
        img = res_img.reshape(1, self.inference_size, self.inference_size, 3)
        img = torch.from_numpy(img.transpose(0, 3, 1, 2))
        img = img.to(device).float() / 255.0    

        # inference
        preds = self.model(img)
        output = handle_preds(preds, device, self.score_threshold)
        return output

    def postprocess(self, ori_img, output):
        LABEL_NAMES = []
        with open(self.class_names_path, 'r') as f:
            for line in f.readlines():
                LABEL_NAMES.append(line.strip())
        
        H, W, _ = ori_img.shape
        scale_h, scale_w = H / 352., W / 352.

        detected_boxes = []
        for box in output[0]:
            box = box.tolist()
           
            obj_score = box[4]
            category = LABEL_NAMES[int(box[5])]
            x1, y1 = int(box[0] * W), int(box[1] * H)
            x2, y2 = int(box[2] * W), int(box[3] * H)
            detected_box = [x1, y1, x2, y2, category]
            detected_boxes.append(detected_box)

            cv2.rectangle(ori_img, (x1, y1), (x2, y2), (255, 255, 0), 2)
            cv2.putText(ori_img, '%.2f' % obj_score, (x1, y1 - 5), 0, 0.7, (0, 255, 0), 2)  
            cv2.putText(ori_img, category, (x1, y1 - 25), 0, 0.7, (0, 255, 0), 2)
        return ori_img, detected_boxes
