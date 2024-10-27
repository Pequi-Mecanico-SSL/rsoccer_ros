import robosim
from typing import List

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist

class SimulatorNode(Node):

    def __init__(self):
        super().__init__('simulator')

        field_type = 2  # 0 for Division A, 1 for Division B, 2 Hardware Challenges
        self.n_robots_blue = 2  # number of blue robots
        self.n_robots_yellow = 2  # number of yellow robots
        time_step_ms = 25  # time step in milliseconds
        # ball initial position [x, y, v_x, v_y] in meters and meter/s
        ball_pos = [0.0, 0.0, 0.0, 0.0]

        # robots initial positions [[x, y, angle], [x, y, angle]...], where [[id_0], [id_1]...]
        # Units are meters and degrees
        blue_robots_pos = [[-0.2, 0.0, 0.0], [-0.4, 0.0, 0.0]]
        yellow_robots_pos = [[0.2, 0.0, 0.0], [0.4, 0.0, 0.0]]

        # Init simulator
        self.sim = robosim.SSL(
            field_type,
            self.n_robots_blue,
            self.n_robots_yellow,
            time_step_ms,
            ball_pos,
            blue_robots_pos,
            yellow_robots_pos,
        )

        self.ball_publisher = self.create_publisher(Pose2D, '/simulator/poses/ball', 10)

        # /simulator/robots_poses/blue/0, /simulator/robots_poses/blue/1, ...
        self.pose_publishers = []
        for i in range(self.n_robots_blue):
            self.pose_publishers.append(
                self.create_publisher(Pose2D, f'/simulator/poses/blue/robot{i}', 10)
            )
        for i in range(self.n_robots_yellow):
            self.pose_publishers.append(
                self.create_publisher(Pose2D, f'/simulator/poses/yellow/robot{i}', 10)
            )
        
        self.latest_robot_actions = [[0.0 for _ in range(6)] for _ in range(self.n_robots_blue + self.n_robots_yellow)]
        
        self.robot_cmd_subscribers = []
        for i in range(self.n_robots_blue):
            self.robot_cmd_subscribers.append(
                self.create_subscription(Twist, f'/simulator/cmd/blue/robot{i}',
                                         lambda msg: self.robot_cmd_callback(i, msg), 10)
            )
        for i in range(self.n_robots_yellow):
            self.robot_cmd_subscribers.append(
                self.create_subscription(Twist, f'/simulator/cmd/yellow/robot{i}',
                                         lambda msg: self.robot_cmd_callback(self.n_robots_blue + i, msg), 10)
            )
    
        timer_period = time_step_ms / 1000.0  # seconds
        self.timer = self.create_timer(timer_period, self.update_state_and_publish)

        self.get_logger().info('Simulator Node Started')

    def robot_cmd_callback(self, i, msg):
        self.latest_robot_actions[i] = [
            0, # has_v_wheel
            msg.linear.x, # wheel_0_speed or v_x
            msg.linear.y, # wheel_1_speed or v_y
            msg.angular.z, # wheel_2_speed or v_angle
            0, # wheel_3_speed (not used because has_v_wheel is 0)
            0, # kick_v_x
            0, # kick_v_y
            0, # dribbler
        ]
    
    def update_state_and_publish(self):
        self.sim.step(self.latest_robot_actions)

        # Units are meters, meters/s, degrees
        # state is [ball_x, ball_y, ball_z, ball_v_x, ball_v_y,
        #           blue_0_x, blue_0_y, blue_0_angle, blue_0_v_x, blue_0_v_y, blue_0_v_angle,
        #           blue_0_infrared, blue_0_desired_wheel0_speed, blue_0_desired_wheel1_speed,
        #           blue_0_desired_wheel2_speed, blue_0_desired_wheel3_speed, ...]
        state = self.sim.get_state()

        ball_msg = Pose2D()
        ball_msg.x = state[0]
        ball_msg.y = state[1]
        ball_msg.theta = 0.0
        self.ball_publisher.publish(ball_msg)
        
        for i in range(self.n_robots_blue):
            msg = Pose2D()
            msg.x = state[5 + i * 11]
            msg.y = state[6 + i * 11]
            msg.theta = state[7 + i * 11]
            self.pose_publishers[i].publish(msg)
        
        for i in range(self.n_robots_yellow):
            msg = Pose2D()
            msg.x = state[5 + self.n_robots_blue * 11 + i * 11]
            msg.y = state[6 + self.n_robots_blue * 11 + i * 11]
            msg.theta = state[7 + self.n_robots_blue * 11 + i * 11]
            self.pose_publishers[self.n_robots_blue + i].publish(msg)


def main(args=None):
    rclpy.init(args=args)

    simulator_node = SimulatorNode()

    rclpy.spin(simulator_node)

    simulator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
