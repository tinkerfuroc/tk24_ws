import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Direction
from cv_bridge import CvBridge
import mediapipe as mp

def cross_product(p1, p2, p3):
    return (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p3[0] - p1[0]) * (p2[1] - p1[1])

def get_p1_p2(elbow_x, elbow_y, wrist_x, wrist_y, w, h):
    p1 = (wrist_x, wrist_y)
    p2 = (0, 0)
    if elbow_x < wrist_x:
        if elbow_y < wrist_y:
            if cross_product((elbow_x, elbow_y), p1, (w, h)) * cross_product((elbow_x, elbow_y), p1, (w, 0)) < 0:
                p2 = (w, p1[1] + (w - p1[0]) * (elbow_y - p1[1]) / float(elbow_x - p1[0]))
            else:
                p2 = (p1[0] + (h - p1[1]) * (elbow_x - p1[0]) / float(elbow_y - p1[1]), h)
        else:
            if cross_product((elbow_x, elbow_y), p1, (w, h)) * cross_product((elbow_x, elbow_y), p1, (w, 0)) < 0:
                p2 = (w, p1[1] + (w - p1[0]) * (elbow_y - p1[1]) / float(elbow_x - p1[0]))
            else:
                p2 = (p1[0] + (0 - p1[1]) * (elbow_x - p1[0]) / float(elbow_y - p1[1]), 0)
    else:
        if elbow_y < wrist_y:
            if cross_product((elbow_x, elbow_y), p1, (0, h)) * cross_product((elbow_x, elbow_y), p1, (0, 0)) < 0:
                p2 = (0, p1[1] + (0 - p1[0]) * (elbow_y - p1[1]) / float(elbow_x - p1[0]))
            else:
                p2 = (p1[0] + (h - p1[1]) * (elbow_x - p1[0]) / float(elbow_y - p1[1]), h)
        else:
            if cross_product((elbow_x, elbow_y), p1, (0, h)) * cross_product((elbow_x, elbow_y), p1, (0, 0)) < 0:
                p2 = (0, p1[1] + (0 - p1[0]) * (elbow_y - p1[1]) / float(elbow_x - p1[0]))
            else:
                p2 = (p1[0] + (0 - p1[1]) * (elbow_x - p1[0]) / float(elbow_y - p1[1]), 0)
    return p1, p2

class PointDirection(Node):

    def __init__(self):
        super().__init__('point_direction')
        self.publisher = self.create_publisher(Direction, 'vision/point_direction', 10)
        timer_period = 1/30  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.subscriber = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.img_callback,
            2)
        self.bridge = CvBridge()
        self.model = mp.solutions.pose.Pose(static_image_mode=False,        # 是静态图片还是连续视频帧
                    model_complexity=1,             # 取0,1,2；0最快但性能差，2最慢但性能好
                    smooth_landmarks=True,          # 是否平滑关键点
                    min_detection_confidence=0.5,   # 置信度阈值
                    min_tracking_confidence=0.5)    # 追踪阈值
        self.results = Direction()

    def timer_callback(self):
        self.publisher.publish(self.results)

    def img_callback(self, data):
        self.results = Direction()
        self.results.header = data.header
        current_frame = self.bridge.imgmsg_to_cv2(data)
        results_pose = self.model.process(current_frame) # 将RGB图片输入模型，获取预测结果
        ## 获取图片长宽
        h1, w1 = current_frame.shape[0], current_frame.shape[1]

        if results_pose.pose_landmarks:
            # 左胳膊肘
            left_elbow_x = int(results_pose.pose_landmarks.landmark[13].x * w1)
            left_elbow_y = int(results_pose.pose_landmarks.landmark[13].y * h1)

            # 右胳膊肘
            right_elbow_x = int(results_pose.pose_landmarks.landmark[14].x * w1)
            right_elbow_y = int(results_pose.pose_landmarks.landmark[14].y * h1)

            # 左手腕
            left_wrist_x = int(results_pose.pose_landmarks.landmark[15].x * w1)
            left_wrist_y = int(results_pose.pose_landmarks.landmark[15].y * h1)

            # 右手腕
            right_wrist_x = int(results_pose.pose_landmarks.landmark[16].x * w1)
            right_wrist_y = int(results_pose.pose_landmarks.landmark[16].y * h1)

            left_p1, left_p2 = get_p1_p2(left_elbow_x, left_elbow_y, left_wrist_x, left_wrist_y, w1, h1)
            self.results.left_p1_x, self.results.left_p1_y = left_p1
            self.results.left_p2_x, self.results.left_p2_y = left_p2

            right_p1, right_p2 = get_p1_p2(right_elbow_x, right_elbow_y, right_wrist_x, right_wrist_y, w1, h1)
            self.results.right_p1_x, self.results.right_p1_y = right_p1
            self.results.right_p2_x, self.results.right_p2_y = right_p2


def main(args=None):
    rclpy.init(args=args)

    point_direction = PointDirection()

    rclpy.spin(point_direction)
    point_direction.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()