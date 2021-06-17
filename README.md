# experiment-for-robot
实验过程中，将所有文件夹放在ros的工作空间下即可
运行过程中 chess_gene是生成所需的棋盘格标定板
具体命令是 rosrun chess_gene chessboard 会在相关路径下生成一个棋盘格标定板
Ar 标签在https://chev.me/arucogen/网址下进行生成


标定kinect2的内参和两个相机的外参关系可以直接通过iai_kinect2包进行
编译后首先输入
roslaunch kinect2_bridge kinect2_bridge.launch
然后展开标定

标定kinect摄像机
rosrun kinect2_bridge kinect2_bridge _fps_limit:=2
得到串口名字
rosrun kinect2_calibration kinect2_calibration chess9x6x0.25 record color
这条指令标定彩色摄像头，按几次空格键保存图片，最好超过十张图片，以保证标定的准确
rosrun kinect2_calibration kinect2_calibration chess9x6x0.25 calibrate color
会生成calib_color.yaml 文件

rosrun kinect2_calibration kinect2_calibration chess9x6x0.25 record ir
标定红外，按几次空格键（原理同上）
rosrun kinect2_calibration kinect2_calibration chess9x6x0.25 calibrate ir
会生成calib_ir.yaml 文件

rosrun kinect2_calibration kinect2_calibration chess9x6x0.25 record sync
帧同步标定按几次空格键
rosrun kinect2_calibration kinect2_calibration chess9x6x0.25 calibrate sync
会生成calib_pose.yaml 文件

rosrun kinect2_calibration kinect2_calibration chess9x6x0.25 record depth

rosrun kinect2_calibration kinect2_calibration chess9x6x0.25 calibrate depth
深度标定此时执行此指令会生成calib_depth.yaml 文件

然后再把calib_color.yaml calib_ir.yaml calib_pose.yaml calib_depth.yaml拷贝到/home/robot/catkin_ws/src/iai_kinect2/kinect2_bridge/data/(相机名称自己要查询，再新建一个文件夹）文件夹中搞定。

标定手眼标定需要 iai_kinect2 aruco_ros eye_hand vision_visp-melodic这三个包
如果运行不起来，https://blog.csdn.net/zhang970187013/article/details/81098175按照这个重新装
用已有的eye_hand代替easy_handeye

以上可以完成相机标定



接下来是坐标使用
robot107 roscore
先在robot107-2上输入 roslaunch kinect2_bridge robot.launch
后面在robot107上输入 roslaunch kinect2_bridge robot.launch
最后在打开新终端
rostopic echo /planning_scene(需要panda107进行界面更新）
