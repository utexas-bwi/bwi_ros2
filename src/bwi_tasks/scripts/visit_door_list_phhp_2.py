import rclpy
 from geometry_msgs.msg import PoseWithCovarianceStamped

class navigation:
    def __init__(self):
        self._sub_amcl_pose = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_cb, 10)
        self.amcl_pose = None

    def amcl_pose_cb(self, msg):
        self.amcl_pose = msg.pose

COORDS = [[-54.58991818711629, 21.194475279584932, 0.07484182869923418],
        [-54.50827572522094, 6.13055453567641, 0.0837730356769868],
        [-61.66703914742318, -10.268698670511574, 0.644691857820201],
        [-31.175605849972044, -8.568029840569146, 0.7233142134679189],
        [-4.96671776219221, -8.53552262842314, 0.9991906542880025],
        [-7.8076208523647646, 20.951359603578094, 0.9913647509614236]]

def main(args=None):
    rclpy.init(args=args)


