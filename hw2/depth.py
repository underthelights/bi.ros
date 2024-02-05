import rospy
from sensor_msgs.msg import LaserScan, Image
import cv2
import numpy as np
from cv_bridge import CvBridge

class ROS_sub():
    def __init__(self,
                 lidar_topic='/hsrb/base_scan',
                 rgb_topic='/hsrb/head_rgbd_sensor/rgb/image_raw',
                 depth_topic='/hsrb/head_rgbd_sensor/depth_registered/image_raw',
                 threshold_dist=0.5):
        self.rgb_sub = rospy.Subscriber(rgb_topic, Image, self._rgb_callback)
        self.depth_sub = rospy.Subscriber(depth_topic, Image, self._depth_callback)
#        self.lidar_sub = rospy.Subscriber(lidar_topic, LaserScan, self._lidar_callback)

        self.threshold_dist = threshold_dist
        self.dist = None
        self.rgb_img = None
        self.depth_img = None
        self.bridge = CvBridge()

    def _rgb_callback(self, data):
        rospy.loginfo('image_sub node started')
#        rospy.loginfo('RGB:{}'.format(type(self.rgb_img)))
        self.rgb_img = cv2.cvtColor(np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1), cv2.COLOR_RGB2BGR)
        if self.dist is not None:
            cv2.putText(self.rgb_img, "Dist to Door:{:.3f}".format(self.dist),(50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 3)
            if self.dist<=1.5:
                cv2.putText(self.rgb_img, "Door Closed.".format(self.dist),(100, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0),3)
            else :
                cv2.putText(self.rgb_img, "Door Opened.".format(self.dist), (100,100), cv2.FONT_HERSHEY_SIMPLEX,1, (0,0,255), 3)
        cv2.imshow("rgb_img", self.rgb_img)

        cv2.waitKey(1)

    def _depth_callback(self, data):
        self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")

        h,w = self.depth_image.shape
        mx, my = w//2, h//2
        roi_size =10

        roi = self.depth_image[my-roi_size:my+roi_size, mx-roi_size:mx+roi_size]
        roi = roi[~np.isnan(roi)]

        if roi.size>0:
            self.dist = np.mean(roi)
        else:
            self.dist = 0
#        rospy.loginfo('Depth:{}'.format(type(self.depth_image)))
        rospy.loginfo('Depth:{}'.format(self.depth_image))

    def _lidar_callback(self, data):
        data_np = np.asarray(data.ranges)
        data_np[np.isnan(data_np)] = 0.0  # remove nans
        num_angles = data_np.shape # resolution of the lidar
        _width = 15 # ranges that you'll search
        center_idx = data_np.shape[0] // 2
        # print(num_angles)

        roi = data_np[center_idx - _width: center_idx + _width]
        self.dist = np.mean(roi)
        rospy.loginfo('Lidar :{}'.format((self.dist)))
#        self.threshold_dist.append(self.dist)
# self.dist = data_np[center_idx]
# print("dist: ", self.dist)


if __name__ == '__main__':
    rospy.init_node('door_detection')
    ls = ROS_sub()# '/hsrb/base_scan','hsrb/head_rgbd_sensor/rgb/image_raw', .5)
    rospy.spin()

#print(self.threshold_dist)
