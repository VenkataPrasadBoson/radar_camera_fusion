#!/usr/bin/env python3
import rospy
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import Twist
from numpy import interp
from std_msgs.msg import Bool


class JoyControl:
    def __init__(self):
        self.steer_angle = 0
        self.speed = 0
        self.break_in = 0
        self.min_steer_angle = int(rospy.get_param("joy/min_steering_angle", 30))
        self.max_steer_angle = int(rospy.get_param("joy/max_steering_angle", -30))
        self.max_speed = rospy.get_param("joy/max_throttle_speed", 2)
        self.ack_msg = AckermannDrive()
        self.ackrem_pub = rospy.Publisher(
            "vehicle/cmd_drive_safe", AckermannDrive, queue_size=10
        )
        rospy.Subscriber("/cmd_vel", Twist, self.joy_callback)
        rospy.Subscriber("/object_detection_status", Bool, self.timer)
        rospy.Timer(rospy.Duration(0.2), self.timer)

    def joy_callback(self, data):
        s = data.linear.x
        # int(interp((self.value), [-25, 25], [40, 180]))
        self.speed = interp(s, (-1, 1), (-self.max_speed, self.max_speed))
        steer = data.angular.z
        self.steer_angle = int(
            interp(steer, [-1, 1], [self.min_steer_angle, self.max_steer_angle])
        )
        

    def timer(self, event):
        self.obs_status = event.data
        if self.obs_status == True:
            
            print("self.steer_angle", self.steer_angle)
            print("speed", self.speed)
            self.ack_msg.speed = self.speed

            self.ack_msg.steering_angle = self.steer_angle
            self.ack_msg.jerk = 0
        else:
            self.ack_msg.jerk = 1
        self.ackrem_pub.publish(self.ack_msg)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("joy_ackremann",anonymous=True)
    j = JoyControl()
    j.run()
