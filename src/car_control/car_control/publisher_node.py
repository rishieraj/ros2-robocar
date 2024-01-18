import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import math as m
import matplotlib.pyplot as plt


class ControlPublisher(Node):

    def __init__(self):
        super().__init__('control_publisher')
        self.sub = self.create_subscription(Float64MultiArray, 'pose_orientation', self.pose_orient_callback, 10)

        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        timer_period = 0.5  # seconds
        self.x_pose = 0
        self.x_lst = []
        self.y_lst = []
        self.yaw_lst = []
        self.steer_control = []
        self.y_pose = 0
        self.steer = 0.0
        self.sub  # prevent unused variable warning
        self.timer = self.create_timer(timer_period, self.control_publisher)
        self.reached = False

    def pose_orient_callback(self, msg):
        self.yaw = msg.data[2]

    def control_publisher(self):
        if self.reached:
            return

        joint_positions = Float64MultiArray()
        wheel_velocities = Float64MultiArray()
        x_t = 10
        y_t = 10
        linear_vel = 2.0
        dt = 0.5
        Vx = 0.095*linear_vel*np.cos(np.pi + self.yaw)
        Vy = 0.095*linear_vel*np.sin(np.pi + self.yaw)
        self.x_pose = self.x_pose + Vx*dt
        self.y_pose = self.y_pose + Vy*dt

        yaw_error = -(3*np.pi/4) - self.yaw
        x_pose_error = (x_t/y_t)*self.y_pose - self.x_pose
        y_pose_error = (y_t/x_t)*self.x_pose - self.y_pose

        self.x_lst.append(x_pose_error)
        self.y_lst.append(y_pose_error)
        self.yaw_lst.append(yaw_error)
        
        
        if (yaw_error <= -0.05) or (yaw_error >= 0.05):
            steer_angle = -yaw_error
        elif (x_pose_error <= -0.5) or (x_pose_error >= 0.5):
            steer_angle = 10*m.atan(x_pose_error/self.y_pose)
        elif (y_pose_error <= -0.5) or (y_pose_error >= 0.5):
            steer_angle = -10*m.atan(y_pose_error/self.x_pose)
        else:
            steer_angle = self.steer

        self.steer_control.append(steer_angle)

        if (x_t-0.5 <= self.x_pose <= x_t+0.5) and (y_t-0.5 <= self.y_pose <= y_t+0.5):
            linear_vel=0.0
            steer_angle=0.0
            self.reached = True
            self.get_logger().info('Destination has been reached!')
            

            # Plotting the errors
            time = np.arange(0, len(self.y_lst) * 0.5, 0.5)

            fig, ax = plt.subplots(1, 2, figsize=(10, 5))

            # Plot 1
            ax[0].plot(time, self.x_lst, label='X-Pose Error')
            ax[0].plot(time, self.y_lst, label='Y-Pose Error')
            ax[0].plot(time, self.yaw_lst, label='Yaw Error')
            ax[0].set_xlabel('Time')
            ax[0].set_ylabel('Errors')
            ax[0].set_title('Error vs Time')
            ax[0].legend()
            ax[0].grid(True)



            #Plot 2
            ax[1].plot(time, self.steer_control, label='Steer Control Command')
            
            ax[1].set_xlabel('Time')
            ax[1].set_ylabel('Steering Control Command')
            ax[1].set_title('Control vs Time')
            ax[1].legend()
            ax[1].grid(True)
            
            plt.xticks(np.arange(min(time), max(time) + 0.5, step=10)) # Set the x-ticks at every 0.5 seconds
            
            # Display the plot
            plt.tight_layout()
            plt.show()


        wheel_velocities.data = [linear_vel,linear_vel,-linear_vel,-linear_vel]
        joint_positions.data = [steer_angle,steer_angle]
        self.steer = steer_angle

        print('The steering angle command is:', steer_angle)

        self.joint_position_pub.publish(joint_positions)
        self.wheel_velocities_pub.publish(wheel_velocities)


def main(args=None):
    rclpy.init(args=args)
    control_publisher = ControlPublisher()
    rclpy.spin(control_publisher)
    control_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()