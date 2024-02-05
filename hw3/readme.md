# [Task #3] Table Edge Calculation

## [Goal] 
- Table Edge를 측정함

## [Solution]
- `sudo apt-get install ros-noetic-ros-numpy``

- initial step
    ```
    roscore
    rosbag play -l hsr_table_length
    ```
    
- Depth Sensor - `_depth_callback` 함수
    - ROI를 추출하여 1-dim image로 재구성
        - h//4:h//2 - h의 중간 지점(1/4 ~ 1/2) / w//4:w*3//4 - w의 중간 부분(1/4 ~ 3/4)
        - 이후 reshape(-1)로 1-dim으로 재구성
    - valid pixel value만 필터링, 이후 filtered value 중 최솟값을 거리로 정의
    - code
        ```python
        import rospy
        from sensor_msgs.msg import LaserScan, Image, PointCloud2
        import cv2
        import numpy as np
        from cv_bridge import CvBridge
        import ros_numpy
        
        # ROS의 여러 센서로부터 데이터 수신을 위해 활용되는 class
        class HSR_sub():
            def __init__(self,
                         lidar_topic='/hsrb/base_scan',
                         rgb_topic='/hsrb/head_rgbd_sensor/rgb/image_raw',
                         depth_topic='/hsrb/head_rgbd_sensor/depth_registered/image_raw',
                         pc_topic='/hsrb/head_rgbd_sensor/depth_registered/rectified_points'):
                
                # subscriber setting for RGB, Depth, LiDAR, pointcloud 
                self.rgb_sub = rospy.Subscriber(rgb_topic, Image, self._rgb_callback)
                self.depth_sub = rospy.Subscriber(depth_topic, Image, self._depth_callback)
                self.lidar_sub = rospy.Subscriber(lidar_topic, LaserScan, self._lidar_callback)
                self._pc_sub = rospy.Subscriber(pc_topic, PointCloud2, self._pc_callback)
        
                self.dist = None
                self.rgb_img = None
                self.depth_img = None
                self.pc = None
                self.bridge = CvBridge()
        
            # RGB Image to BGR Format (OpenCV) 처리 - 받은 data를 image로 변환, putText를 통해 화면에 표시
            def _rgb_callback(self, data):
                # 1. log self.dist : 현재 저장된 distance 정보
                rospy.loginfo(self.dist) 
                # 2. RGB image data를 BGR Format으로 변환 (OpenCV image 처리)
                #  np.frombuffer : binary to NumPy Array - memory buffer로부터 image sensor binary data를 받아 8bit unsigned int array로 해석 (0-255 range의 image data)
                # .reshape(data.height, data.width, -1): 1-dim array to image dimension = 3dim array 
                #     마지막 차원 -1 : NumPy가 나머지 차원을 자동으로 계산 - 3dim RGB channel이므로 channel의 수를 자동으로 계산하여 배열을 재구성
                self.rgb_img = cv2.cvtColor(np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1), cv2.COLOR_RGB2BGR)
                # 3. putText를 통한 테이블 가장자리 길이 표시
                cv2.putText(self.rgb_img, "Table edge length: {fname}".format(fname=self.dist/1000), \
                            (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, \
                            (0, 255, 0), 3)
                # 4. 이미지 화면상 표시, key event 대기-이미지를 보여주고 사용자의 입력을 기다리는 OpenCV의 패턴
                cv2.imshow("rgb_img", self.rgb_img)
                cv2.waitKey(1)
        
            # Depth Image Data 처리 - 특정 영역의 depth 계산 후 최소거리 계산
            def _depth_callback(self, data):
                # self.bridge : ROS img message - OpenCV Image 사이 변환을 쉽게 도와주는 class 
                # 32FC1 encoding : 32bit floating point with single precision 
                # 이를 self.depth_image에 저장하여 깊이 정보를 사용할 준비를 마친 것입니다. 변환된 이미지는 각 픽셀의 깊이 값을 포함하며, 이를 사용하여 로봇이 주변 환경의 3차원 구조를 이해하고 상호 작용하는 데 사용할 수 있습니다.\
                self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1") # [mm]
        
                w, h = 640, 480
                # ROI를 추출하여 1-dim image로 재구성
                # h//4:h//2 - h의 중간 지점(1/4 ~ 1/2) / w//4:w*3//4 - w의 중간 부분(1/4 ~ 3/4)
                roi = self.depth_image[h//4:h//2, w//4:w*3//4].reshape(-1)
                # valid pixel value만 필터링
                roi_inline = roi[roi>0.01]
                # filtered value 중 최솟값을 거리로 정의
                self.dist = roi_inline.min()
        
            # point cloud data : 3-dim 공간에서의 점들의 집합 -> 물체의 위치와 형태 파악
            def _pc_callback(self, data):
                self.pc = ros_numpy.numpify(data)
        
            # LiDAR : 로봇 주변 물체들과의 거리정보 확인
            def _lidar_callback(self, data):
                data_np = np.asarray(data.ranges) # Lidar Data to Numpy Array
                data_np[np.isnan(data_np)] = 0.0  # remove nans
                num_angles = data_np.shape # resolution of the lidar - 각도에 따라 측정한 거리 값들의 배열
                _width = 15 # ranges that you'll search
                center_idx = data_np.shape[0] // 2 # 360도로 스캔하는 LiDAR에 대해 중앙 값을 찾음
                # print(num_angles)
        
        if __name__ == '__main__':
            # ROS Node 초기화
            rospy.init_node('table_edge_length_detection')
            hsr_sub = HSR_sub()
            rate = rospy.Rate(5)
            image_resolution = (480, 640, 3)
            
            # while loop 내부에서 sensor data 지속적으로 처리
            while not rospy.is_shutdown():
                if hsr_sub.pc is None:
                    continue
                pc = np.array(hsr_sub.pc.tolist()).reshape(image_resolution[0], image_resolution[1], -1) #[m]
                rate.sleep()
        ```
        
    
- LiDAR - `_lidar_callback` 함수
    1. LiDAR sensor로부터 받은 data 처리 - 최근접한 object까지의 distance 계산
    2. 이후 클래스의 인스턴스 변수 `self.dist`에 저장합니다.
    - code
        
        ```python
        import rospy
        from sensor_msgs.msg import LaserScan, Image, PointCloud2
        import cv2
        import numpy as np
        from cv_bridge import CvBridge
        import ros_numpy
        
        class HSR_sub():
            def __init__(self,
                         lidar_topic='/hsrb/base_scan',
                         rgb_topic='/hsrb/head_rgbd_sensor/rgb/image_raw',
                         depth_topic='/hsrb/head_rgbd_sensor/depth_registered/image_raw',
                         pc_topic='/hsrb/head_rgbd_sensor/depth_registered/rectified_points'):
                self.rgb_sub = rospy.Subscriber(rgb_topic, Image, self._rgb_callback)
                self.depth_sub = rospy.Subscriber(depth_topic, Image, self._depth_callback)
                self.lidar_sub = rospy.Subscriber(lidar_topic, LaserScan, self._lidar_callback)
                self._pc_sub = rospy.Subscriber(pc_topic, PointCloud2, self._pc_callback)
        
                self.dist = None
                self.rgb_img = None
                self.depth_img = None
                self.pc = None
                self.bridge = CvBridge()
        
            def _rgb_callback(self, data):
                self.rgb_img = cv2.cvtColor(np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1), cv2.COLOR_RGB2BGR)
                cv2.putText(self.rgb_img, "Table edge length: ".format(self.dist), \
                            (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, \
                            (0, 255, 0), 3)
                cv2.imshow("rgb_img", self.rgb_img)
                cv2.waitKey(1)
        
            def _depth_callback(self, data):
                self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1") # [mm]
        
            def _pc_callback(self, data):
                self.pc = ros_numpy.numpify(data)
        
            def _lidar_callback(self, data):
                data_np = np.asarray(data.ranges)  # Lidar Data to Numpy Array
                data_np[np.isnan(data_np)] = 0.0   # remove nans
        
                # Assume we are looking for an object at a certain range
                # A simple threshold on distance to detect objects close to the robot (like the edge of a table)
                threshold_distance = 0.8  # 80cm, for example
                close_objects = data_np < threshold_distance
        
                if np.any(close_objects):
                    # Find the closest object
                    min_distance = np.min(data_np[close_objects])  # Find the minimum distance in the filtered close objects
                    self.dist = min_distance  # Update the self.dist with the distance of the closest object
        
                    # You can print or log the distance of the closest object
                    print(f"Close object detected at distance: {self.dist} meters")
                    # Further processing can be done here, like calculating the exact position of the edge or responding to the object
                else:
                    # No close objects detected, you might want to set self.dist to None or keep the previous value
                    # depending on how you want to handle this scenario.
                    # For example, setting it to None:
                    self.dist = None
        
        if __name__ == '__main__':
            rospy.init_node('table_edge_length_detection')
            hsr_sub = HSR_sub()
            rate = rospy.Rate(5)
            image_resolution = (480, 640, 3)
            while not rospy.is_shutdown():
                if hsr_sub.pc is None:
                    continue
                pc = np.array(hsr_sub.pc.tolist()).reshape(image_resolution[0], image_resolution[1], -1) #[m]
                rate.sleep()
        ```