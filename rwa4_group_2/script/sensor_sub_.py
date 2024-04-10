#!/usr/bin/env python3

from rwa4_group_2.sensor_sub_interface import ImageSubscriber
import rclpy


def main():
    rclpy.init(args=None)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()