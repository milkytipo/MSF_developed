#MSF_developed
---
### An extended version which regarded the original ETH MSF (mult-sensor fusion) as the lib to import, and extracted the *msf_updates* into an independent section. This version realized fusing IMU, SLAM (ORB_SLAM2) and GNSS (or GPS) into a framework.
---

Authors: [Zida Wu](https://github.com/milkytipo), [Zheng Gong](https://github.com/gauxonz), [Qiang Liu](https://github.com/LugiaLiu)

MSF is a good algorithm, but obviously, the code and the theory are really not friendly to a researcher. I think that's why there are few people want to develop it further. So I just summarize my experience, hoping the later people could run it as soon as possbile. MSF is good algorithm, if you can run it well, it must give you some insightful ideas about the sensor fusion.

ETH的MSF是一个非常优秀的多源传感器融合（松耦合）开源框架，尽管是松耦合，但是最终经过修改和调试后的MSF的效果依然让我觉得非常惊喜。同时由于MSF原始代码非常的不优美简洁，所以我特地将MSF源码作为一个“库”来使用，而将我们可以扩展和开发的部分单独提取出来开发成MSF_developed，这样对代码理解和使用会有更好的帮助。需要提前说明的是，由于原始MSF文章公式的错误和代码的问题，导致研究者者直接阅读代码和看文章公式推导会存在疑惑，这里我都将一一解答。此外，鉴于KITTI数据集本身的缺陷，以及跑MSF的需要等原因，要想完整跑出理想MSF效果，需要KITTI数据预处理、MSF源码修改，GPS加噪声（可选），ORBSLAM2的publish位姿的话题（topic），轨迹显示等各项操作，不过我已经全部一条龙在本Readme中进行说明和解释，Good Luck！如果觉得有用，给个star吧！hhhhhh

In order to make it convenient for researcher to understand and run the *MSF_developed* framework (because the original version is a little hard to use), I have to claim something about that at first. The details about those will be discussed one by one.
> * You have to preprocess the KITTI dataset, since [the KITTI dataset and its kitti to bag tools][3] have some "bugs".
> * [Original MSF paper][1] has some mistakes about equations, it will probably puzzle you especially when you run the MSF algorithm at the same time. What's more, the symbols (such as *q_wv*, *q_ic*) in the MSF code don't follow the roles in his papers which also perplexs researchers.
> * The code in original MSF is not concise enough, so I put the MSF as the lib and develop the *MSF_developed* to make users fully master how to run the framework. Moreover, I have fixed some bugs and time setting in MSF (if you use MSF firstly you have large probability get troubles about the timestamp), so I recommmend you use my [ethzasl_msf][2] as your MSF choice.
> * You have to fully understand parameters in the [position_pose_sensor_fix.yaml][yaml], since I think it contains the core idea of the MSF design.

Okay, now I will discuss the problems one bye one. I will tell you how to fix it and which tools you should use. After those a brief demo would be given to show how to run the **MSF_developed**.

## **Key Problem**
### 1.KITTI Dataset Process and Add noise into GPS data
**Problems:**
> * KITTI has ***synced*** and ***unsynced*** data. The synced data has low frequency but the unsynced data owns high IMU Hz.
> * KITTI dataset's data (mainly about oxts(GPS+IMU) ) is not continuous, which indicates that there are many data lost in the process
> * The GPS data in KITTI was obtained by RTK-fixed results, which owns high positioning quality. If we want to demonstrate the affect of the GPS in our framework, we should not induce such good results.
> * The kitti to rosbag tools which officially recommended are not good, because it will lost covariance in GPS and other problems.

Thus, we should find a way to synchronize the IMU-GPS-Images data, interprete the lost data to make it continous, and add some noise into the GPS data. If possible, we should directly read data from files, but not from bag just like VINS-Mono. (Actually, I realize this funcion in [read_dataset ](https://github.com/milkytipo/MSF_developed/tree/master/src/read_dataset)node.

**Tools：**
[KITTI-Interpre][4]
After using this tool, you can transfer the unsynced data into synced data with high IMU frequency, as well as continous GPS and IMU. Besides, in the [InterpAndGnss.m](https://github.com/gauxonz/KITTI-interp/blob/master/InterpAndNoiseGnss.m) you can add noise in the GPS.

Please note that, the dataset I used follows the files structure follows this tools instruction. The reason why I mentioned it is that, in my improvement I directly reading raw data from files but not through rosplay bag.

### 2. The theory imperfection in original MSF paper and MSF code
**Problems:**
> * The theory in [Original MSF paper][1] is insightful, but the Jacobi matrixs and other equations in papers are wrong, those in the author's PhD thesis paper are not right neither.
> *  As for the diffrent representation of the symbols between code and paper, you should only notice one thing: that is the *q_wv* and *p_wv* is the opposite to the original meaning in papers. The normal representation is , for example, the *q_ic* means that to project the IMU position into camera coordinates. However, the *q_wv* in code is not like that. 

**Solutions:**
At first, the figure shown below is about the whole architecture about msf. 
<div align = center><img width = "630" height ="400" src ="https://github.com/milkytipo/MSF_developed/blob/master/images/MSF_Geometry_Constraint.png" /></div>

We can sperately introduce the SLAM-IMU sub-framework and GPS-IMU sub-framework. Let's see the Jacobi matrix and the geometry constraint as follows.

<div align = center><img width = "630" height ="400" src ="https://github.com/milkytipo/MSF_developed/blob/master/images/SLAM-IMU_Frame.png" />
<img width = "630" height ="400" src ="https://github.com/milkytipo/MSF_developed/blob/master/images/GPS-IMU_Frame.png" /></div>

If you have comprehended the MSF and Error-state Kalman Filter well, you can understand each iterm in Jacobi meaning what. For example, as you can see, the SLAM observation in SLAM-IMU framework can calibrate every error-state except for Δb.

However!!! That is not like that!!! ***Let's take SLAM-IMU as example, from the Jacobi matrix , you can use /slam/pose to calibrate the *q_ic* and *q_iw* (in msf code, *q* means *q_iw*), but if you have a further consideration it will not work. Suppose the /slam/pose has a drift, it could come from the *q_ic* or *q_iw*, but the equation will calibrate the two parameters together, so it will change the two transforms at the same time even if there's only the drift of the *q_iw*. ***
(if you would had run the code well, you can have a test to demenstrate that: if you loose fix of the q_ic， or loose fix of the q_wv， the final result would be bad than the fixed result)

Luckly, we can have offline calibration about Tic and Tig, thus, let's fix the transform of Tic and Tig. As for the Twv, I will talk about that below~

Further, if you want to fuse the GPS-IMU-SLAM together, you should seriously think about the *p_wv* and *q_wv*. Of course, as we talk above, one /slam/pose observation will calibrate the Twv and Tiw together, and there is no direct observation of *w* node in the geometry constraint. It will results in unpredictable drifts. However, the two things you have to know.

**Firstly, GPS-IMU framework is about the global framework (or ENU framework), but the SLAM-IMU is the local framework (or SLAM framework), the accociation about the two framework is the w-v transform, so you have to loose the relationship to let the observation slowly calibrate it.**

**Secondly, there is indeed an implicit observation in the *w* node---gravity. Actually, the IMU sensors based on the gravity-alignment coordinates. The gravity can be regarded as the observation of the *w* point. So, the only problem of the calibration is about the yaw drift (since gravity can not calibrate the yaw drift.**

### 3. Important Parameters 
Three important files
> * msf_types.h contains the gravity setting
> * msf_core.cfg contains the **range** of IMU noises, which means you cannot set unlimited IMU noise values in .yaml
> * position_pose_sensor_fix.yaml contains other important paramters setting. Even I think it is one of the utmost things if you want to run MSF.

As for the [position_pose_sensor_fix.yaml][yaml], I think I have added essential annotation in this file, you can should carefully read them.

The only thing I should mention is about the IMU. If the dataset doesn't provide the noise parameters or you want to deploy the msf into you own robot, you should get those parameters by yourself. So, I recommended the [kalibr_allam](https://github.com/rpng/kalibr_allan) to obtain those. Be friendly, in case you haven't learned it well about Allan variance, I want to provide other information about the Allan variance.

Generally, you should use the angle/velocity random walk ( or "white nosie") to set the *core_noise_acc*, and use the  (angle/velocity) rate random walk to set the *core_noise_accbias*. ( I used the noise name from the original Allan paper ["Analysis and Modeling of Inertial Sensors Using Allan Variance"](https://ieeexplore.ieee.org/document/4404126), in other tools, they called the angular random walk into "while noise", and call rate random walk into "bias" or "random walk". Seriously, I think they are not rigorous. Additionally, if you cannot obain the rate random walk, you can use the bias instability to replace it.

Parameter values are very very important for MSF, be careful!Z

## **MSF_developed Method Structure**
MSF_developed
----|include
    --------|gps_sensor_handler
    ---------------------|position_sensorhandler.hpp/.h
    ---------------------|position_measurement.h
    --------|vision_sensor_handle
    ---------------------|pose_sensorhandler.hpp/.h
    ---------------------|pose_measurement.h
----|src
    --------|position_pose_msf (*position_msf* means only GPS-IMU, *pose_msf* means only SLAM-IMU)
    ---------------------|position_pose_sensormanager.h   

**_sensornamager.h** is used for reading parameters from yaml, initialization, as well as creating the pose/position handler (as /include/_hanlder.h) and IMU_handler (as /msf_core/../msf_IMUHandler_ROS.h).

p.s.1 IMU_handler is in charge of Kalman prediction process.

p.s.2 Initialization is called by the method msf_core::Init() (in the msf_core_inl.h) and MSF_InitMeasurement() (in msf_measurement_inl.h).

**pose_hanlder.h** is used to subscribe topics and transfer the topics, add measurement into queue (by calling methond in msf_core).

**position_hanlder.h** is the same function as the pose_hanlder.h, but it has one more function is to transfer the latitude/longitude/altitude into ENU coordinates. 

**_measurement.h** is in charege of calculating the Jacobi H matrix (by .Apply() method) and residual r matrix. Further, to accomplish the Kalman update process. (by .CalculateAndApplyCorrection() method)

There is an important class which you should notice: [msf_core::MSF_Core](http://ethz-asl.github.io/ethzasl_msf/html/classmsf__core_1_1MSF__Core.html), which is in charge of the IMU pose information, measurement management and the state buffer management.

**debug tips**:
1. if you want to check whether the GPS sensor fusion or SLAM sensor fusion are working on well or not, you can directly cancel the annotation of the MSF_WARN_STREAM("\***GPS H Matrix is calculating***") in position_measurement.h and the MSF_WARN_STREAM("\***SLAM H Matrix is calculating***") in pose_measurement.h.
2. if you want to obtain the real time core state display, you just need to cancel the annotation of the the MSF_INFO_STREAM("the fixed pwv :"<< **xxxxxx**) in pose_measurement.h.

(I know the debug way is a little stupid... but they are simple as well as efficient.)

## **KITTI Example**

### Prerequisite
1. [ORB_SLAM2](https://github.com/milkytipo/ORB_SLAM2) which can publish the its transform (pose and position) topic. The core idea about this is add the publishment topic in ros_stereo.cc
2. Since the MSF_developed directly read data from files, so your **files structure** shoud follow the rules as [KITTI-Interpre][4]. (Actually it is the same as the *Rawdata* you download from KITTI). Of course you can use KITTI bag, if you read the source code of the [*read_dataset*](https://github.com/milkytipo/MSF_developed/tree/master/src/read_dataset) node you will find they are the same as rosplay bag.

### Run MSF_developed and Visualization

```
 roslaunch ai_robot_lcsfl KITTI_start.launch
```
After the MSF and ORB_SLAM2 have all started:

```
rosrun ai_robot_lcsfl read_node /(your_root_path)/KITTI/RawDataFixed/2011_10_03/2011_10_03_drive_0027 noised-01
```
**Note:** I named the noised file as noised-01, you can change the GPS noised setting and generate your own noised data. Of course, you can use raw GPS data.
```
rosbag record /imu /gps/fix /msf_core/pose /slam/tf
```
Keep the bag files in order to plot the trajectory.

------
### **P.S. 1 A Simple Visualization Tool for SLAM/MSF/GPS trajectory - plotTrajectory**

I developed a simple tool to plot the trajectory, called [plotTrajectory](https://github.com/milkytipo/plotTrajectory)

> 1. set the file path in the loadtxt("/path") in extract_topics_from_rosbag.py
> 2. python extract_topics_from_rosbag.py
> 3. python plotTrajectory.py

Then, you can see the trajectory like this:
<div align = center><img width = "600" height ="400" src ="https://github.com/milkytipo/MSF_developed/blob/master/images/MSF-SLAM-GPS.png" /></div>
<div align = center><img width = "600" height ="400" src ="https://github.com/milkytipo/MSF_developed/blob/master/MSF_noised_results/noised03-GPS-MSF-Front.png" /></div>

### **P.S. 2 A Error Calculator Tool for SLAM/MSF/GPS - evo**
If you want to obtain the RMSE, STD, MAE and other error items, I recommend you the [evo](https://github.com/MichaelGrupp/evo) tool. In addition, I have transfered the KITTI groudtruth file into a general format for MSF error evaluation, namely [kitti_00_groudtruth.tum](https://github.com/milkytipo/MSF_developed/blob/master/MSF_noised_results/kitti_00_groudtruth.tum). 

**Note:** I recommed you use this groudtruth file, in case you spend lots of time to do the same work as I have done.

After you have run the MSF_developed algorithm and recorded the /slam/tf, /msf_core/pose, /imu and /gps/fix topics, you can use evo to estimate the error levels.

First, save the topics in bag into .tum. Use the command below to general *pose.tum* file.
```
evo_traj bag MSF_developed_fusion.bag /msf_core/pose --save_as_tum
```
Second, evaluate the error between kitti_00_groudtruth.tum and pose.tum.
```
evo_ape tum kitti_00_groudtruth.tum pose.tum -v -a -p
```
```
evo_rpe tum kitti_00_groudtruth.tum pose.tum -v -a -p
```
I use the [KITTI-Interpre][4] to generate a noised-03 oxts, then the msf results are as follow:
<div align = center><img width = "600" height ="400" src ="https://github.com/milkytipo/MSF_developed/blob/master/MSF_noised_results/noised03-APE.png" /></div>
<div align = center><img width = "600" height ="400" src ="https://github.com/milkytipo/MSF_developed/blob/master/MSF_noised_results/noised02-APE-MAP.png" /></div>

### Noised-03 APE RMSE excel

| Type        | MSF_developed   |  GPS-noised  | ORB_SLAM2  |
| --------   |:----:  | :----:  |:----:  |
| RMSE     | 9.219306 |   TODO   | TODO   |
| STD       |  5.023872   |  TODO   | TODO   |
| Mean       |  7.730220   |  TODO   | TODO   |

**Note:** I have tested four noised GPS conditions, and the results are contained in the [MSF_noised_results](https://github.com/milkytipo/MSF_developed/tree/master/MSF_noised_results)

[1]: https://ieeexplore.ieee.org/document/6696917/authors#authors
[2]: https://github.com/milkytipo/ethzasl_msf
[3]: http://www.cvlibs.net/datasets/kitti/raw_data.php
[4]: https://github.com/gauxonz/KITTI-interp
[yaml]:https://github.com/milkytipo/MSF_developed/blob/master/position_pose_sensor_fix.yaml


