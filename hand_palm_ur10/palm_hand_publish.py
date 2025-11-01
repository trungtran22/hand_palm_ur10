
#!/home/trungtran/catkin_ws/ros_venv/bin python3.12
import rclpy
from rclpy.node import Node
import cv2
import mediapipe as mp
from cv_bridge import CvBridge # Bridge between ROS and OpenCV
from sensor_msgs.msg import Image # Image message type
from geometry_msgs.msg import Point # Coordinates message type

class PalmDetectorNode(Node):
    def __init__(self):
        super().__init__('palm_detector_node')
        
        # Initialize MediaPipe Hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            model_complexity=0,
            max_num_hands=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5)
        
        # Initialize ROS
        self.bridge = CvBridge()
        
        # Subscribe to the image topic
        self.image_subscription = self.create_subscription(
            Image,
            '/image',  
            self.image_callback_bb, # <<< Change function here if needed
            10)
        
        # Publisher for palm position
        self.palm_publisher = self.create_publisher(
            Point,
            '/hand_palm_position', # <<< Tên topic chúng ta tự đặt
            10)
            
        self.get_logger().info('Palm Detector Node đã khởi động và sẵn sàng!')
        self.get_logger().info("Đang chờ ảnh trên topic '/image'...")

    # Using wrist and middle finger MCP for palm coordinates
    def image_callback(self, msg):
    
        try:
            # 1. Convert ROS Image to OpenCV Image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Không thể chuyển đổi ảnh: {e}')
            return

        # Retrieve image dimensions
        image_height, image_width, _ = cv_image.shape

        # 2. Process image with MediaPipe
        # Flip if needed
        # cv_image = cv2.flip(cv_image, 1) 
        
        # Convert BGR to RGB for MediaPipe
        image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        image_rgb.flags.writeable = False
        
        results = self.hands.process(image_rgb)

        # 3. Calculate palm center
        palm_point = Point()
        
        # Default values if no hand is detected
        palm_point.x = 0.0
        palm_point.y = 0.0
        palm_point.z = 0.0 

        if results.multi_hand_landmarks:
            hand_landmarks = results.multi_hand_landmarks[0]
            landmark_list = hand_landmarks.landmark

            # Retrieve wrist [0] and middle finger MCP landmarks [9]
            wrist_landmark = landmark_list[self.mp_hands.HandLandmark.WRIST]
            mcp_landmark = landmark_list[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP]

            # Pixel coordinates
            wrist_x = wrist_landmark.x * image_width
            mcp_x = mcp_landmark.x * image_width
            wrist_y = wrist_landmark.y * image_height
            mcp_y = mcp_landmark.y * image_height
            
            # Palm center as midpoint between wrist and middle finger MCP
            palm_center_x = (wrist_x + mcp_x) / 2.0
            palm_center_y = (wrist_y + mcp_y) / 2.0

            # Assign to Point message
            palm_point.x = palm_center_x
            palm_point.y = palm_center_y
            palm_point.z = 1.0 #fixed value indicating hand detected
            
            # (Optional) Draw palm center for debugging
            # cv2.circle(cv_image, (int(palm_center_x), int(palm_center_y)), 20, (0, 0, 255), -1)

        # (Optional) Show debug image
        # cv2.imshow("Palm Detector", cv_image)
        # cv2.waitKey(1)

        # 5. Publish the Point message
        self.palm_publisher.publish(palm_point)

    # Using bounding box method for palm coordinates including z_coor as area
    def image_callback_bb(self, msg):
        """
        Callback này được gọi mỗi khi có một tin nhắn Image mới.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Không thể chuyển đổi ảnh: {e}')
            return

        image_height, image_width, _ = cv_image.shape

        # Image processing with MediaPipe
        image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        image_rgb.flags.writeable = False
        results = self.hands.process(image_rgb)
        image_rgb.flags.writeable = True

        palm_point = Point()
        palm_point.x = 0.0
        palm_point.y = 0.0
        palm_point.z = 0.0 

        if results.multi_hand_landmarks:
            hand_landmarks = results.multi_hand_landmarks[0]
            landmark_list = hand_landmarks.landmark

            # 1. Find bounding box
            all_x_coords = [lm.x for lm in landmark_list]
            all_y_coords = [lm.y for lm in landmark_list]
            
            # convert to pixel coordinates
            min_x_px = min(all_x_coords) * image_width
            max_x_px = max(all_x_coords) * image_width
            min_y_px = min(all_y_coords) * image_height
            max_y_px = max(all_y_coords) * image_height

            # 2. Box width, height, area
            bbox_width = max_x_px - min_x_px
            bbox_height = max_y_px - min_y_px
            bbox_area = bbox_width * bbox_height

            # 3. Find center of bounding box as palm center
            palm_center_x = (min_x_px + max_x_px) / 2.0
            palm_center_y = (min_y_px + max_y_px) / 2.0


            # 4. Assign to Point message
            palm_point.x = palm_center_x 
            palm_point.y = palm_center_y
            palm_point.z = bbox_area 
            
            # Optional: Draw bounding box for debugging
            # cv2.rectangle(cv_image, (int(min_x_px), int(min_y_px)), (int(max_x_px), int(max_y_px)), (0, 255, 0), 2)
        
        # (Optional) Show debug image
        # cv2.imshow("Palm Detector", cv_image)
        # cv2.waitKey(1)

        # 5. Publish the Point message
        self.palm_publisher.publish(palm_point)


def main(args=None):
    rclpy.init(args=args)
    
    palm_detector_node = PalmDetectorNode()
    
    rclpy.spin(palm_detector_node)
    
    # Cleanup
    palm_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()