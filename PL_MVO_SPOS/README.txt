"PL_MVO" by Haoang Li (haoang.li@whu.edu.cn),
refer to and cite accordingly : Haoang Li, Jian Yao*, Xiaohu Lu and Junlin Wu, Combining Points and Lines
for Camera Pose Estimation and Optimization in Monocular Visual Odometry, Submitted to The 2017 IEEE/RSJ
International Conference on Intelligent Robots and Systems (IROS 2017), March 2017.
Only for academic or other non-commercial purposes.

1. Running Preparation
The program is developed with Visual_Studio_2013 on Win64 system, which was organized with a 
script file named "CMakeLists". Besides, it is built on the open source library : OpenCV 2.4.9 and Eigen 3.
Before you use the project, you should mind the following items : 
(1) Make sure the OpenCV and Eigen library is installed on your computer.
(2) Open the CMakeLists located in the Folder 'Src' with any text editor to specify the directory of 
Library Eigen, e.g. SET(Eigen_DIR "C:\\Program Files\\Eigen\\eigen3")
(3) Open the CMakeLists with Cmake (a software for organizing project) and configure and generate the 
VS2013 project. Note that 'OpenCV_DIR' should be specify as install path of OpenCV, e.g. 
C:/Program Files/opencv/build. See more information please visit web: https://cmake.org/

2. Usage
The Entry "main()" can be found at "PL_MVO_SPOS.cpp". After running the program (If there are a error, 
please try to remove ALL_BUILD and ZERO_CHECK, only reserving PL_MVO_SPOS), the screen will display: 
a) true rotaiton matrix and translation vector; b) optimization result by proposed optimization strategy.

If you have any questions, please don't hesitate to contact us !

                                           Computer Vision & Remote Sensing (CVRS) Lab, Wuhan University 
                                                                                              2017.03.05