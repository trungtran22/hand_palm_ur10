#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point  # Để nhận (subscribe)
from std_msgs.msg import String      # Để gửi (publish)

# --- CÁC HẰNG SỐ CẦN HIỆU CHỈNH ---
# (Giống như trước, bạn cần hiệu chỉnh các giá trị này)
NEUTRAL_X = 170
NEUTRAL_Y = 90
NEUTRAL_AREA = 25000  # << BẠN PHẢI THAY ĐỔI GIÁ TRỊ NÀY

# Hệ số khuếch đại (Scaling)
# (Chúng ta cần đổi dấu Z_SCALE vì speedl X là tiến/lùi)
Z_SCALE = 0.0000025
X_SCALE = -0.0005
Y_SCALE = -0.0005

# Tốc độ tối đa (m/s)
MAX_VEL = 0.2
# --- KẾT THÚC CÁC HẰNG SỐ ---

# Các hằng số cho lệnh URScript
ACCELERATION = 0.5  # Gia tốc (m/s^2)
LOOKAHEAD_TIME = 0.1  # Thời gian (s)


class PalmToScriptNode(Node):
    def __init__(self):
        super().__init__('palm_to_script')
        
        # Subscriber để nhận tọa độ pixel/diện tích
        self.hand_subscription = self.create_subscription(
            Point,
            '/hand_palm_position',
            self.palm_callback,
            10)
        
        # Publisher để gửi lệnh URScript
        self.script_publisher = self.create_publisher(
            String,
            '/urscript_interface/script_command', # <<< TOPIC ĐÚNG
            10)

        self.get_logger().info('Node Palm-to-Script đã khởi động.')
        self.get_logger().info(f'Điểm nghỉ: X={NEUTRAL_X}, Y={NEUTRAL_Y}, Area={NEUTRAL_AREA}')
        self.last_command_was_stop = False # Biến cờ để tránh spam lệnh "stop"


    def palm_callback(self, msg):
        x_center = msg.x
        y_center = msg.y
        area = msg.z

        script_cmd = String()

        # Nếu không phát hiện tay (diện tích = 0), gửi lệnh dừng
        if area == 0.0:
            if not self.last_command_was_stop:
                # Gửi lệnh stopl (dừng di chuyển tuyến tính)
                script_cmd.data = f"stopl({ACCELERATION})"
                self.script_publisher.publish(script_cmd)
                self.last_command_was_stop = True
            return

        # Nếu thấy tay, đặt lại cờ
        self.last_command_was_stop = False

        # 1. Tính toán lỗi (Error)
        z_error = area - NEUTRAL_AREA
        x_error = x_center - NEUTRAL_X
        y_error = y_center - NEUTRAL_Y

        # 2. Áp dụng hệ số (Gain) để chuyển đổi sang vận tốc (m/s)
        # speedl: [vx, vy, vz, rx, ry, rz]
        # (Lưu ý: Hệ tọa độ của robot có thể khác với camera)
        # vel_x (speedl X - tiến/lùi)
        vel_x = Z_SCALE * z_error
        # vel_y (speedl Y - trái/phải)
        vel_y = X_SCALE * x_error
        # vel_z (speedl Z - lên/xuống)
        vel_z = Y_SCALE * y_error
        
        # 3. Giới hạn vận tốc (Saturation)
        vel_x = max(-MAX_VEL, min(vel_x, MAX_VEL))
        vel_y = max(-MAX_VEL, min(vel_y, MAX_VEL))
        vel_z = max(-MAX_VEL, min(vel_z, MAX_VEL))
        
        # 4. Tạo chuỗi lệnh URScript
        # speedl([vx,vy,vz, rx,ry,rz], gia_toc, thoi_gian)
        cmd_string = f"speedl([{vel_x},{vel_y},{vel_z},0,0,0], {ACCELERATION}, {LOOKAHEAD_TIME})"
        script_cmd.data = cmd_string

        # 5. Gửi lệnh
        self.script_publisher.publish(script_cmd)


def main(args=None):
    rclpy.init(args=args)
    palm_to_script_node = PalmToScriptNode()
    
    try:
        rclpy.spin(palm_to_script_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Gửi lệnh dừng khi tắt node
        palm_to_script_node.get_logger().info('Đang gửi lệnh dừng...')
        stop_cmd = String()
        stop_cmd.data = f"stopl({ACCELERATION})"
        palm_to_script_node.script_publisher.publish(stop_cmd)
        
        palm_to_script_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()