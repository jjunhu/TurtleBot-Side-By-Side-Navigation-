#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class SideBySideNavigator:
    def __init__(self):
        rospy.init_node('side_by_side_navigator', anonymous=True)

        # Publishers for controlling the robots
        self.robot1_cmd_vel = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=10)
        self.robot2_cmd_vel = rospy.Publisher('/robot2/cmd_vel', Twist, queue_size=10)

        # Subscribers to robot sensor data
        rospy.Subscriber('/robot1/scan', LaserScan, self.robot1_sensor_callback)
        rospy.Subscriber('/robot2/scan', LaserScan, self.robot2_sensor_callback)

        self.robot1_obstacle_distance = float('inf')
        self.robot2_obstacle_distance = float('inf')

        self.rate = rospy.Rate(10)  # 10 Hz

    def robot1_sensor_callback(self, data):
        # Process sensor data for Robot 1
        self.robot1_obstacle_distance = min(data.ranges)

    def robot2_sensor_callback(self, data):
        # Process sensor data for Robot 2
        self.robot2_obstacle_distance = min(data.ranges)

    def run(self):
        while not rospy.is_shutdown():
            # Navigation logic and velocity command publishing
            velocity_command1 = Twist()
            velocity_command2 = Twist()

            # Check for obstacles and adjust speed for Robot 1
            if self.robot1_obstacle_distance < 0.5:
                velocity_command1.linear.x = 0.0  # Stop if obstacle too close
            else:
                velocity_command1.linear.x = 0.2  # Move forward otherwise

            # Check for obstacles and adjust speed for Robot 2
            if self.robot2_obstacle_distance < 0.5:
                velocity_command2.linear.x = 0.0
            else:
                velocity_command2.linear.x = 0.2

            # Synchronize the robots' speeds
            # Adjust speed of robot 2 to match robot 1 for side-by-side navigation
            velocity_command2.linear.x = velocity_command1.linear.x

            self.robot1_cmd_vel.publish(velocity_command1)
            self.robot2_cmd_vel.publish(velocity_command2)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        navigator = SideBySideNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass
