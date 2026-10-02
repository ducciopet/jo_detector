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
from ultralytics import YOLO
from PIL import Image as PILImage, ImageDraw, ImageFont


path_curr = os.path.dirname(__file__)

def msg_to_cv2_fallback(msg):
    """Fallback conversion from ROS Image message to OpenCV image"""
    try:
        if msg.encoding == "bgr8":
            img_data = np.frombuffer(msg.data, dtype=np.uint8)
            img = img_data.reshape((msg.height, msg.width, 3))
            return np.ascontiguousarray(img)
        elif msg.encoding == "rgb8":
            img_data = np.frombuffer(msg.data, dtype=np.uint8)
            img = img_data.reshape((msg.height, msg.width, 3))
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            return np.ascontiguousarray(img)
        else:
            return None
    except Exception as e:
        print(f"Fallback conversion error: {e}")
        return None
device = "cuda" if torch.cuda.is_available() else "cpu"

class yolo_detector(Node):
    def __init__(self):
        super().__init__('yolov11_detector')
        print("[onboardDetector]: yolo detector init...")

        self.image_topic = str(self.declare_parameter('image_topic', '').value)
        self.detected_image_topic = str(self.declare_parameter('detected_image_topic', '').value)
        self.detected_bboxes_topic = str(self.declare_parameter('detected_bboxes_topic', '').value)
        self.inference_time_topic = str(self.declare_parameter('inference_time_topic', '').value)
        self.weights_path = str(self.declare_parameter('weights_path', '').value)
        self.class_names_path = str(self.declare_parameter('class_names_path', '').value)
        self.target_classes = list(self.declare_parameter('target_classes', ['person']).value)
        self.score_threshold = float(self.declare_parameter('score_threshold', 0.7).value)
        self.timer_period_sec = float(self.declare_parameter('timer_period_sec', 0.033).value)
        self.inference_size = int(self.declare_parameter('inference_size', 352).value)
        self.queue_size = int(self.declare_parameter('queue_size', 10).value)

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
        self.use_half = (device == "cuda")

        self.img_received = False
        self.img_detected = False

        # init and load
        self.model = YOLO(self.weights_path)
        self.model.eval()
        self.get_logger().info(
            f"YOLOv11 configured: device={device}, half={self.use_half}, image_topic={self.image_topic}, "
            f"weights={self.weights_path}, score_threshold={self.score_threshold:.2f}"
        )

        # subscriber. rclpy/image_transport has no C++-style transport-hint
        # helper, so we subscribe directly to the image_transport-mangled
        # "<base_topic>/compressed" topic (published automatically by any
        # image_transport::Publisher-based driver, e.g. realsense2_camera,
        # as long as compressed_image_transport is installed — see
        # docker/Dockerfile).
        self.br = CvBridge()
        self.img_sub = self.create_subscription(
            CompressedImage, self.image_topic + '/compressed', self.image_callback, self.queue_size)

        # publisher
        self.img_pub = self.create_publisher(Image, self.detected_image_topic, self.queue_size)
        self.bbox_pub = self.create_publisher(Detection2DArray, self.detected_bboxes_topic, self.queue_size)
        self.time_pub = self.create_publisher(Float64, self.inference_time_topic, 1)

        # timers
        self.create_timer(self.timer_period_sec, self.detect_callback)
        self.create_timer(self.timer_period_sec, self.vis_callback)
        self.create_timer(self.timer_period_sec, self.bbox_callback)
    
    def image_callback(self, msg):
        try:
            # JPEG/PNG-decode the compressed payload. cv2.imdecode returns
            # BGR, which matches what the old raw-Image path produced after
            # its manual rgb8->BGR swap, so everything downstream (inference,
            # vis_callback's "bgr8" republish) is unaffected.
            compressed = np.frombuffer(msg.data, dtype=np.uint8)
            decoded = cv2.imdecode(compressed, cv2.IMREAD_COLOR)
            if decoded is None:
                raise ValueError(f"cv2.imdecode failed (format='{msg.format}', {len(msg.data)} bytes)")

            self.img = np.ascontiguousarray(decoded, dtype=np.uint8)

            # Validation
            assert isinstance(self.img, np.ndarray), f"Not numpy array: {type(self.img)}"
            assert self.img.dtype == np.uint8, f"Wrong dtype: {self.img.dtype}"
            assert self.img.flags['WRITEABLE'], "Array is not writable"

            self.img_received = True
            if not hasattr(self, '_first_image_logged'):
                h, w = self.img.shape[:2]
                self.get_logger().info(f"First image received on {self.img_sub.topic_name} ({w}x{h}, format={msg.format})")
                self._first_image_logged = True

        except Exception as e:
            self.get_logger().error(f"Image callback error: {type(e).__name__}: {e}")
            self.img_received = False

    def detect_callback(self):
        start_time = time.time()
        if self.img_received and hasattr(self, 'img') and self.img is not None:
            try:
                # Debug logging
                self.get_logger().debug(f"Before inference - type: {type(self.img)}, dtype: {self.img.dtype}, shape: {self.img.shape}, writable: {self.img.flags['WRITEABLE']}, C_CONTIGUOUS: {self.img.flags['C_CONTIGUOUS']}")
                
                output = self.inference(self.img)
                self.detected_img, self.detected_bboxes = self.postprocess(self.img, output)
                self.img_detected = True
            except Exception as e:
                self.get_logger().error(f"Error in inference: {type(e).__name__}: {e}")
                import traceback
                self.get_logger().error(traceback.format_exc())
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
                    # detected_box format: [x1, y1, x2, y2, class, score]
                    x1 = float(detected_box[0])
                    y1 = float(detected_box[1])
                    x2 = float(detected_box[2])
                    y2 = float(detected_box[3])
                    class_name = str(detected_box[4])
                    score = float(detected_box[5])
                    # Store top-left corner in hypothesis pose, class in class_id, and size in bbox
                    from vision_msgs.msg import ObjectHypothesisWithPose
                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id = class_name
                    hyp.hypothesis.score = score
                    hyp.pose.pose.position.x = x1  # top-left x
                    hyp.pose.pose.position.y = y1  # top-left y
                    bbox_msg.results.append(hyp)
                    bbox_msg.bbox.size_x = float(abs(x2 - x1))  # width
                    bbox_msg.bbox.size_y = float(abs(y2 - y1))  # height
                    bboxes_msg.detections.append(bbox_msg)
            bboxes_msg.header.stamp = self.get_clock().now().to_msg()
            self.bbox_pub.publish(bboxes_msg)

    def inference(self, ori_img):
        try:
            # Final safety conversion - reconstruct array completely to ensure it's pure numpy
            ori_img = np.array(ori_img, dtype=np.uint8, copy=True, order='C')
            
            # Verify it's actually a numpy array
            if not isinstance(ori_img, np.ndarray):
                raise TypeError(f"After conversion, still not numpy array: {type(ori_img)}")
            
            if ori_img.dtype != np.uint8:
                ori_img = ori_img.astype(np.uint8)
            
            # Ensure C-contiguous
            if not ori_img.flags['C_CONTIGUOUS']:
                ori_img = np.ascontiguousarray(ori_img, dtype=np.uint8)
            
            # Validate shape
            if len(ori_img.shape) != 3 or ori_img.shape[2] != 3:
                raise ValueError(f"Invalid image shape: {ori_img.shape}. Expected (H, W, 3)")
            
            self.get_logger().debug(f"In inference - final type: {type(ori_img)}, dtype: {ori_img.dtype}, shape: {ori_img.shape}, C_CONTIGUOUS: {ori_img.flags['C_CONTIGUOUS']}")
            
            # Use PIL for image resizing instead of cv2 to avoid numpy/OpenCV compatibility issues
            try:
                pil_img = PILImage.fromarray(ori_img, mode='RGB')
                pil_img = pil_img.resize((self.inference_size, self.inference_size), PILImage.Resampling.BILINEAR)
                res_img = np.array(pil_img, dtype=np.uint8)
            except Exception as e:
                self.get_logger().warning(f"PIL resize failed: {e}. Trying torch...")
                # Fallback to torch resizing
                img_tensor = torch.from_numpy(ori_img.copy()).permute(2, 0, 1).unsqueeze(0).float()
                img_resized = torch.nn.functional.interpolate(img_tensor, size=(self.inference_size, self.inference_size), mode='bilinear', align_corners=False)
                res_img = img_resized.squeeze(0).permute(1, 2, 0).numpy().astype(np.uint8)
            
            # Ensure res_img is a proper numpy array before torch conversion
            res_img = np.array(res_img, dtype=np.uint8, copy=True)
            res_img = np.ascontiguousarray(res_img)
            
            # Reshape and convert to tensor
            img = res_img.reshape(1, self.inference_size, self.inference_size, 3)
            # Convert to float32 for model input
            img = img.astype(np.float32)
            # Use torch.tensor() instead of torch.from_numpy() to avoid numpy version conflicts
            img_tensor = torch.tensor(img, dtype=torch.float32)
            # Transpose: (batch, height, width, channels) -> (batch, channels, height, width)
            img_tensor = img_tensor.permute(0, 3, 1, 2)
            img_tensor = img_tensor.to(device) / 255.0   

            # inference
            preds = self.model(img_tensor, device=device, half=self.use_half)[0]
            return [preds.boxes.xyxyn, preds.boxes.conf, preds.boxes.cls]
            
        except Exception as e:
            self.get_logger().error(f"Inference failed - INPUT type={type(ori_img).__name__}, dtype={getattr(ori_img, 'dtype', 'N/A')}, shape={getattr(ori_img, 'shape', 'N/A')}, error: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            raise

    def postprocess(self, ori_img, output):
        # Ensure ori_img is a proper numpy array
        ori_img = np.array(ori_img, dtype=np.uint8, copy=True)
        ori_img = np.ascontiguousarray(ori_img)
        
        LABEL_NAMES = []
        with open(self.class_names_path, 'r') as f:
            for line in f.readlines():
                LABEL_NAMES.append(line.strip())
        
        H, W, _ = ori_img.shape

        detected_boxes = []
        
        # Convert to PIL for drawing to avoid cv2 numpy compatibility issues
        pil_img = PILImage.fromarray(ori_img, mode='RGB')
        draw = ImageDraw.Draw(pil_img)
        
        try:
            # Try to use a default font, fall back to default if not available
            font = ImageFont.load_default()
        except:
            font = None
        
        for i, box in enumerate(output[0]):
            box = box.tolist()
           
            obj_score = float(output[1][i])
            if obj_score < self.score_threshold:
                continue
            category = LABEL_NAMES[int(output[2][i])]
            x1, y1 = int(box[0] * W), int(box[1] * H)
            x2, y2 = int(box[2] * W), int(box[3] * H)
            detected_box = [x1, y1, x2, y2, category, obj_score]
            detected_boxes.append(detected_box)

            # Draw rectangle using PIL (yellow)
            draw.rectangle([x1, y1, x2, y2], outline=(255, 255, 0), width=2)
            
            # Draw text using PIL (green)
            score_text = '%.2f' % obj_score
            draw.text((x1, y1 - 25), category, fill=(0, 255, 0), font=font)
            draw.text((x1, y1 - 5), score_text, fill=(0, 255, 0), font=font)
        
        # Convert back to numpy array
        ori_img = np.array(pil_img, dtype=np.uint8)
        return ori_img, detected_boxes
