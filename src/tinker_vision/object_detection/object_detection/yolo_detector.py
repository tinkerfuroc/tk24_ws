import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox, Object, YoloResults
from cv_bridge import CvBridge
from ultralytics import YOLO

time_period = 1/30
    
class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.publisher = self.create_publisher(YoloResults, "/vision/yolo_results", 2)
        self.timer = self.create_timer(time_period, self.timer_callback)
        self.subscriber = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.img_callback,
            2)
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        self.results = YoloResults()

    def timer_callback(self):
        self.publisher.publish(self.results)


    def img_callback(self, data):
        self.results = YoloResults()
        box = BoundingBox()
        object = Object()
        current_frame = self.bridge.imgmsg_to_cv2(data)
        detection = self.model.predict(current_frame,verbose = False)[0]
        boxes = detection.boxes
        for i in range(len(boxes)):
            box.xmin, box.ymin, box.xmax, box.ymax = boxes[i].xyxy[0].tolist()
            object.prob = boxes[i].conf[0].item()
            object.id = int(boxes[i].cls[0].item())
            object.name = detection.names[object.id]
            object.box = box
            self.results.header = data.header
            self.results.objects.append(object)


def main(args=None):
    rclpy.init(args=args)

    yolo_detector = YoloDetector()

    rclpy.spin(yolo_detector)
    yolo_detector.destroy_node()
    rclpy.shutdown()

  
if __name__ == '__main__':
  main()