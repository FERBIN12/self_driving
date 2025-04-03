from rclpy.node import Node
from std_msgs.msg import String
import serial
import rclpy

class Simple_Serial_traansmitter(Node):
    def __init__(self):
        super().__init__("simple_serial_transmitter")

        # declaring the baudrate and port as params
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)

        self.port_ = self.get_parameter("port").value
        self.baud_rate = self.get_parameter("baud_rate").value

        self.arduino_ = serial.Serial(port= self.port_, baudrate= self.baud_rate, timeout= 0.1)
        self.sub_ = self.create_subscription(String, "serial_transmitter",self.msg_cb, 10)


    def msg_cb(self, msg):
        self.arduino_.write(msg.data.encode("utp-8"))

def main():
    rclpy.init()

    simple_publisher = Simple_Serial_traansmitter()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()