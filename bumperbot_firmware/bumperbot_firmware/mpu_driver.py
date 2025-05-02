#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus

'''These are the register address for the MPU6050'''
PWR_MGMT_1= 0x6B
SMPLRT_DIV= 0x19
CONFIG= 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE= 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47
DEVICE_ADDRESS = 0x68

class MPU6050_driver(Node):
    def __init__(self):
        super().__init__("mpu_driver")

        # Corrected publisher creation
        self.imu_pub_ = self.create_publisher(Imu, "imu/out", 10)

        self.imu_msg_ = Imu()
        self.imu_msg_.header.frame_id = "base_footprint"

        "is_connected bool variable is used to keep track of the IMU status"
        self.is_connected_ = False
        self.bus_ = None  # Initialize bus_ to None
        self.init_i2c()

        # for every 0.01th sec(i.e 100Hz) we are calling a timer_callback 
        self.frequency_ = 0.01  # the imu data is received at 125HZ so setting ours to 100 reduces load on the cpu
        self.timer_ = self.create_timer(self.frequency_, self.timer_cb)

    def init_i2c(self):
        try:
            self.bus_ = smbus.SMBus(1)
            self.bus_.write_byte_data(DEVICE_ADDRESS, SMPLRT_DIV,7)
            self.bus_.write_byte_data(DEVICE_ADDRESS, PWR_MGMT_1, 1)
            self.bus_.write_byte_data(DEVICE_ADDRESS, CONFIG,0)
            self.bus_.write_byte_data(DEVICE_ADDRESS, GYRO_CONFIG, 24)
            self.bus_.write_byte_data(DEVICE_ADDRESS, INT_ENABLE, 1)
            self.is_connected_ = True
        except OSError:
            self.is_connected_ = False
            self.get_logger().error("Failed to initialize I2C connection.")

    def timer_cb(self):
        try:    
            if not self.is_connected_:
                self.init_i2c()

            if self.bus_ is None:
                self.get_logger().error("I2C bus is not initialized.")
                return

            '''Now we gotta get the accelerometer (x,y,z) and gyro (x,y,z) to publish them as ros_imu msg'''
            acc_x = self.read_raw_data(ACCEL_XOUT_H)
            acc_y = self.read_raw_data(ACCEL_YOUT_H)
            acc_z = self.read_raw_data(ACCEL_ZOUT_H)

            gyro_x = self.read_raw_data(GYRO_XOUT_H)
            gyro_y = self.read_raw_data(GYRO_YOUT_H)
            gyro_z = self.read_raw_data(GYRO_ZOUT_H)

            self.imu_msg_.linear_acceleration.x = acc_x / 1670.13
            self.imu_msg_.linear_acceleration.y = acc_y / 1670.13
            self.imu_msg_.linear_acceleration.z = acc_z / 1670.13

            self.imu_msg_.angular_velocity.x = gyro_x / 7509.55
            self.imu_msg_.angular_velocity.y = gyro_y / 7509.55
            self.imu_msg_.angular_velocity.z = gyro_z / 7509.55

            self.imu_msg_.header.stamp = self.get_clock().now().to_msg()

            self.imu_pub_.publish(self.imu_msg_)
        except OSError:
            self.is_connected_ = False
            self.get_logger().error("I2C communication error during reading.")

    def read_raw_data(self, addr):
        high = self.bus_.read_byte_data(DEVICE_ADDRESS, addr)
        low = self.bus_.read_byte_data(DEVICE_ADDRESS, addr+1)

        value = ((high << 8) | low)

        if value > 32767:
            value = value - 65536
        return value

def main():
    rclpy.init()
    mpu_driver = MPU6050_driver()
    rclpy.spin(mpu_driver)
    mpu_driver.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
