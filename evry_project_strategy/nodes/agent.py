#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from evry_project_plugins.srv import DistanceToFlag
class Robot:
    def __init__(self, robot_name):
        self.speed = 0.0
        self.angle = 0.0
        self.sonar = 0.0 
        self.x, self.y = 0.0, 0.0
        self.yaw = 0.0
        self.robot_name = robot_name

        rospy.Subscriber(self.robot_name + "/sensor/sonar_front",
                        Range, self.callbackSonar)
        rospy.Subscriber(self.robot_name + "/odom",
                        Odometry, self.callbackPose)
        self.cmd_vel_pub = rospy.Publisher(
            self.robot_name + "/cmd_vel", Twist, queue_size=1)

    def callbackSonar(self, msg):
        self.sonar = msg.range

    def get_sonar(self):
        return self.sonar

    def callbackPose(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quaternion = msg.pose.pose.orientation
        quaternion_list = [quaternion.x,
                          quaternion.y, quaternion.z, quaternion.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion_list)
        self.yaw = yaw

    def get_robot_pose(self):
        return self.x, self.y, self.yaw

    def constraint(self, val, min=-2.0, max=2.0):
        if val < min:
            return min
        if val > max:
            return max
        return val

    def set_speed_angle(self, linear, angular):
        cmd_vel = Twist()
        cmd_vel.linear.x = self.constraint(linear)
        cmd_vel.angular.z = self.constraint(angular, min=-1, max=1)
        self.cmd_vel_pub.publish(cmd_vel)

    def getDistanceToFlag(self):
        rospy.wait_for_service('/distanceToFlag')
        try:
            service = rospy.ServiceProxy('/distanceToFlag', DistanceToFlag)
            pose = Pose2D()
            pose.x = self.x
            pose.y = self.y
            result = service(pose, int(self.robot_name[-1]))
            return result.distance
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)


class PIDController:
    def __init__(self, kp=0.5, ki=0.1, kd=0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.sum_error = 0.0
        self.prev_error = 0.0
        
    def compute(self, error):
        self.sum_error += error
        diff_error = error - self.prev_error
        output = self.kp * error + self.ki * self.sum_error + self.kd * diff_error
        self.prev_error = error
        return output

def run_demo():
    robot_name = rospy.get_param("~robot_name")
    robot = Robot(robot_name)
    print(f"Robot : {robot_name} is starting..")
    
    pid = PIDController(0.5, 0.1, 0.1)
    
    while not rospy.is_shutdown():
        distance = float(robot.getDistanceToFlag())
        obst_distance = float(robot.get_sonar())
        distance_threshold = 0.5
        
        # PID control for velocity
        velocity = pid.compute(distance - distance_threshold)
        angle = 0
        
        # Basic obstacle avoidance
        if obst_distance < 0.5:
            velocity = 0
        
        if distance < distance_threshold:
            velocity = 0
            
        robot.set_speed_angle(velocity, angle)
        rospy.sleep(0.5)

if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Controller", anonymous=True)
    run_demo()
