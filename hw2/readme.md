# [Task #2] Door Open Detection

## [Goal] 
- python script를 실행하면 동영상에서 문이 열렸을때 Door Open, 문이 닫혔을 때는 Door Closed라는 텍스트가 보여지도록 하기

## [Solution]
- [hw2_door.py](./door_open_detection.py)

1. `roscore` 실행 이후 `rosbag` loop
    - `rostopic` 적용
2. `python hw2_door_open.py` 의 skeleton code 실행
    1. rgb, depth, lidar 데이터 형식이 어떻게 되는지 출력해가면서 확인해보자       
        - RGB - <class ‘numpy.ndarray’>            
        - Depth - <class ‘numpy.ndarray’>
        - Lidar - <class ‘numpy.float64’>
    2. 문이 열렸음을 탐지하기 위해서는 어떤 센서의 데이터가 적합할까?
        - Lidar : 얼마나 문이 열려있는지에 대하여 distance를 측정하는 것이 intuitive하고, 그 threshold를 선정함에 따라 문이 열리고 닫히는 것을 detect할 수 있다.
    3. self.threshold_dist 에 측정된 로봇과 문 사이의 거리가 들어오도록 한다
