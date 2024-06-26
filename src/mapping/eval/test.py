import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from utfr_msgs.msg import ConeMap, EgoState, PoseGraphData
from eufs_msgs.msg import ConeArrayWithCovariance
from nav_msgs.msg import Odometry

class MinimalSubscriber(Node):
  def __init__(self):
      super().__init__('minimal_subscriber')
      self.subscription = self.create_subscription(
          ConeMap,
          'mapping/cone_map',
          self.listener_callback,
          10)
      self.subscription

      self.pose_cb = self.create_subscription(
          EgoState,
          'ekf/pose',
          self.pose_callback,
          10)
      self.pose_cb

      self.gt_pose = self.create_subscription(
          Odometry,
          '/ground_truth/odom',
          self.gt_pose_cb,
          10)
      self.gt_pose

      self.ground_truth = self.create_subscription(
          ConeArrayWithCovariance,
          '/ground_truth/cones_map',
          self.gt_cb,
          10)
      self.ground_truth

      self.slam_pose = self.create_subscription(
          PoseGraphData,
          'mapping/slam_pose',
          self.slam_pose_cb,
          10)
      self.slam_pose

      self.timer = self.create_timer(0.5, self.timer_callback)

      self.cones = []
      self.gt_cones = []
      self.pose = None
      self.gt_pose = None
      self.slam_pose = None
      self.total_error = 0
      self.count = 0
      self.slam_error = 0

  def listener_callback(self, msg):
      self.cones = msg

  def gt_cb(self, msg):
    if self.gt_cones != []:
      return
    
    for type in [msg.yellow_cones, msg.blue_cones, msg.big_orange_cones, msg.orange_cones]:
      for cone in type:
        self.gt_cones.append([cone.point.x, cone.point.y])

  def pose_callback(self, msg):
    self.pose = msg

  def gt_pose_cb(self, msg):
    self.gt_pose = msg
  
  def slam_pose_cb(self, msg):
    self.slam_pose = msg

  def timer_callback(self):

    if self.cones == []:
      return
    
    total = 0
    count = 0
    bad_cones = 0
    for type in [self.cones.left_cones, self.cones.right_cones, self.cones.large_orange_cones, self.cones.small_orange_cones]:
      for cone in type:
        count += 1
        pos_x, pos_y = cone.pos.x, cone.pos.y
        min_dist = 100000
        for row in self.gt_cones:
          dist = ((pos_x - row[0])**2 + (pos_y - row[1])**2)**0.5
          min_dist = min(min_dist, dist)
        total += min_dist
        if min_dist > 0.15:
          bad_cones += 1
    print("Average error: ", (total/count*100), "cm. Bad cones: ", bad_cones, " out of ", count)

    gt_x, gt_y = self.gt_pose.pose.pose.position.x, self.gt_pose.pose.pose.position.y

    x, y = self.pose.pose.pose.position.x, -self.pose.pose.pose.position.y
    error = ((x - gt_x)**2 + (y - gt_y)**2)**0.5
    self.total_error += error

    slam_x, slam_y = self.slam_pose.x, -self.slam_pose.y
    slam_error = ((slam_x - gt_x)**2 + (slam_y - gt_y)**2)**0.5
    self.slam_error += slam_error
    
    self.count += 1
    print("Average EKF position error: ", (self.total_error/self.count*100), "cm")
    print("Average SLAM position error: ", (self.slam_error/self.count*100), "cm")



def main(args=None):

  rclpy.init(args=args)

  minimal_subscriber = MinimalSubscriber()

  rclpy.spin(minimal_subscriber)

  minimal_subscriber.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()