import cv2
import numpy as np
import mediapipe as mp
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class HandGestureBotController(Node):
    def __init__(self):
        super().__init__('hand_gesture_bot_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_cmd)
        self.command = Twist()
        
        # Initialize MediaPipe Hands
        self.mp_hands = mp.solutions.hands
        self.mp_draw = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.7)
        
        # Open the default camera
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error("Error: Could not open webcam.")
            exit()

    def count_fingers(self, hand_landmarks):
        fingers = [0, 0, 0, 0, 0]  # Thumb, Index, Middle, Ring, Pinky
        finger_tips = [4, 8, 12, 16, 20]
        finger_mcp = [2, 5, 9, 13, 17]
        
        for i in range(1, 5):  # Checking fingers other than thumb
            if hand_landmarks.landmark[finger_tips[i]].y < hand_landmarks.landmark[finger_mcp[i]].y:
                fingers[i] = 1
        
        # Thumb detection (depends on hand orientation)
        if hand_landmarks.landmark[finger_tips[0]].x > hand_landmarks.landmark[finger_mcp[0]].x:
            fingers[0] = 1
        
        return sum(fingers)

    def process_gesture(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture frame")
            return
        
        # Convert to RGB
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self.hands.process(rgb_frame)
        text = ""
        
        if result.multi_hand_landmarks:
            for hand_landmarks in result.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                finger_count = self.count_fingers(hand_landmarks)
                
                if finger_count == 1:
                    text = "Start"
                    self.command.linear.x = 0.5
                    self.command.angular.z = 0.0
                elif finger_count == 2:
                    text = "Left"
                    self.command.linear.x = 0.0
                    self.command.angular.z = 0.5
                elif finger_count == 3:
                    text = "Right"
                    self.command.linear.x = 0.0
                    self.command.angular.z = -0.5
                elif finger_count == 4:
                    text = "Reverse"
                    self.command.linear.x = -0.5
                    self.command.angular.z = 0.0
                elif finger_count == 5:
                    text = "Stop"
                    self.command.linear.x = 0.0
                    self.command.angular.z = 0.0
                
                # Display text
                cv2.putText(frame, text, (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Display the frame
        cv2.imshow('Hand Gesture Control', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            self.destroy_node()
            rclpy.shutdown()
    
    def publish_cmd(self):
        self.process_gesture()
        self.publisher_.publish(self.command)
        
def main():
    rclpy.init()
    simple_publisher = HandGestureBotController()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
