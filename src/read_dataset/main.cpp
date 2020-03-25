//
// Created by wuzida on 2020/3/5.
//
#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <cv_bridge/cv_bridge.h>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/LocalCartesian.hpp>

using namespace std;
using namespace Eigen;

#define SEQ_END 9999999999
#define GPS_DIV_FREQ  100 //used to control the GPS and IMU freq
#define SENSOR_SLEEP_TIME 15 //7 //used to control the sleep time between two loop

Eigen::Matrix3d c1Rc0, c0Rc1;
Eigen::Vector3d c1Tc0, c0Tc1;

std::string SEASON_START_TIME;

enum eSensorType{CAM, IMU, GNSS, IMU_GNSS};
int main(int argc, char** argv) {
    auto start_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream ss;
    ss << std::put_time(std::localtime(&start_time), "%FT%T");
    SEASON_START_TIME = ss.str();

    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    ros::Publisher pubLeftImage = n.advertise<sensor_msgs::Image>("/camera/left/image_raw",1000);
    ros::Publisher pubRightImage = n.advertise<sensor_msgs::Image>("/camera/right/image_raw",1000);

    ros::Publisher pubImu = n.advertise<sensor_msgs::Imu>("/imu",1000);
    ros::Publisher pubOrignGnss = n.advertise<sensor_msgs::NavSatFix>("/gps/fix",1000);

    if(argc != 3)
    {
    printf("please intput: rosrun vins kitti_odom_test [config file] [data folder] [oxts type]\n"
    "for example: rosrun vins kitti_odom_test "
    "~/catkin_ws/src/VINS-Fusion/config/kitti_odom/kitti_config00-02.yaml "
    "/media/tony-ws1/disk_D/kitti/odometry/sequences/00/ \n");
    return 1;
    }

    //    string config_file = argv[1];
    //    printf("config_file: %s\n", argv[1]);
    string sequence = argv[1];
    printf("read sequence: %s\n", argv[1]);
    string dataPath = sequence + "/";

    string oxtsType = argv[2];
    printf("oxts type: %s\n", argv[2]);

    list< pair<double,eSensorType> > pairTimeTypeList;
    {
    // load image list
    list<double> imageTimeList;
    {
    FILE *file;
    file = std::fopen((dataPath + "image_00/timestamps.txt").c_str(), "r");
    if (file == NULL)
    {
    printf("cannot find file: %simage_00/timestamps.txt \n", dataPath.c_str());
    ROS_BREAK();
    return 0;
    }
    double imageTime;

    int year, month, day;
    int hour, minute;
    double second;
    while (fscanf(file, "%d-%d-%d %d:%d:%lf", &year, &month, &day, &hour, &minute, &second) != EOF)
    {
    //printf("%lf\n", second);
    #ifdef _img_time_sync_bias_
    imageTimeList.push_back(hour * 60 * 60 + minute * 60 + second + _img_time_sync_bias_);
    #else
    imageTimeList.push_back(hour * 60 * 60 + minute * 60 + second );
    #endif
    }
    std::fclose(file);
    }
    // load gps-imu list
    list<double> imu_gpsTimeList;
    {
    FILE *file;
    file = std::fopen((dataPath + "oxts-" + oxtsType + "/timestamps.txt").c_str(), "r");
    if (file == NULL)
    {
    printf("cannot find file: %soxts-%s/timestamps.txt \n", dataPath.c_str(),oxtsType.c_str());
    ROS_BREAK();
    return 0;
    }
    int year, month, day;
    int hour, minute;
    double second;
    while (fscanf(file, "%d-%d-%d %d:%d:%lf", &year, &month, &day, &hour, &minute, &second) != EOF)
    {
    //printf("%lf\n", second);
    imu_gpsTimeList.push_back(hour * 60 * 60 + minute * 60 + second);
    }
    std::fclose(file);
    }
    // build data orign seq
    auto imgItr(imageTimeList.begin()), imu_gpsItr(imu_gpsTimeList.begin());
    while (imgItr != imageTimeList.end() && imu_gpsItr != imu_gpsTimeList.end() )
    {
    if( imgItr != imageTimeList.end() && *imgItr < *imu_gpsItr)
    {
    pairTimeTypeList.emplace_back(make_pair(*imgItr++, CAM));
    }
    else if (imu_gpsItr != imu_gpsTimeList.end())
    {
    pairTimeTypeList.emplace_back(make_pair(*imu_gpsItr++, IMU_GNSS));
    }
    }
    }

    //   readParameters(config_file);
    //   //estimator.setParameter();
    //   estimator.setParameterOnly();
    //  estimator.processThread_swt = true;
    //    estimator.startProcessThread();
    //   registerPub(n);

    //   FILE* outFile;
    //    outFile = fopen((OUTPUT_FOLDER + "/gvins-fe-traj_" + SEASON_START_TIME + ".txt").c_str(),"w");
    //   if(outFile == NULL)
    //       printf("Output path dosen't exist: %s\n", OUTPUT_FOLDER.c_str());

    //    FILE* tmp_outFile;
    //    tmp_outFile = fopen("/media/joey/dataset/KITTI/RawDataUnsync/2011_10_03/2011_10_03_drive_0027_extract/00_unsync_time.txt","w");
    //    if(tmp_outFile == NULL)
    //        printf("Output path dosen't exist: %s\n", OUTPUT_FOLDER.c_str());
    //
    //    //fprintf(tmp_outFile, "time,ax,ay,az,af,al,au,wx,wy,wz,wf,wl,wu\n");
    //    fprintf(tmp_outFile, "unsync_time\n");

    string leftImagePath, rightImagePath;
    cv::Mat imLeft, imRight;

    const int imu_begin_idx(300);
    const int imu_end_idx(SEQ_END);

    const int img_begin_idx(0);
    const int img_end_idx(SEQ_END);

    int imageIdx(0),imu_gnssIdx(0);
    int first_img_idx = -1;
    const int gnss_ex_div(GPS_DIV_FREQ); // The extrinsic freq divider. As the KITTI GNSS is got from 100hz from oxts,
    // the freq is too high (100hz), we divided it by this $gnss_ex_div to make it lower

    for (auto itPairTimeType = pairTimeTypeList.begin(); itPairTimeType!=pairTimeTypeList.end(); ++itPairTimeType)
    {
        if(ros::ok())
        {
        if ( itPairTimeType->second == CAM )
        {
        // load img
        stringstream ss;
        ss << setfill('0') << setw(10) << imageIdx++;
        if (imu_gnssIdx > imu_begin_idx &&
                imageIdx > img_begin_idx &&
        (next(itPairTimeType)==pairTimeTypeList.end() || next(itPairTimeType)->second!=CAM))
        {
        if (first_img_idx == -1) first_img_idx = imageIdx-1;
        leftImagePath = dataPath + "image_00/data/" + ss.str() + ".png";
        rightImagePath = dataPath + "image_01/data/" + ss.str() + ".png";

        imLeft = cv::imread(leftImagePath, CV_LOAD_IMAGE_GRAYSCALE);
        sensor_msgs::ImagePtr imLeftMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8",
                                                             imLeft).toImageMsg();
        imLeftMsg->header.stamp = ros::Time(itPairTimeType->first);

        imRight = cv::imread(rightImagePath, CV_LOAD_IMAGE_GRAYSCALE);
        sensor_msgs::ImagePtr imRightMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8",
                                                              imRight).toImageMsg();
        imRightMsg->header.stamp = ros::Time(itPairTimeType->first);
        pubLeftImage.publish(imLeftMsg);
        pubRightImage.publish(imRightMsg);

        //                fprintf(tmp_outFile, "%f\n",
        //                        itPairTimeType->first);
        //ROS_INFO("Input Image, time:%f, seq:%d", itPairTimeType->first, imageIdx);
        //                   estimator.inputImage(itPairTimeType->first, imLeft, imRight);
        }
        }
        else if (itPairTimeType->second == IMU_GNSS)// load gps
        {
        stringstream ss;
        ss << setfill('0') << setw(10) << imu_gnssIdx++;
        if (imu_gnssIdx > imu_end_idx || imageIdx>img_end_idx) break;
        if (first_img_idx>-1)
        {
        FILE *GPSFile;
        string GPSFilePath = dataPath + "oxts-"+ oxtsType + "/data/" + ss.str() + ".txt";
        GPSFile = std::fopen(GPSFilePath.c_str(), "r");
        if (GPSFile == NULL)
        {
        printf("cannot find file: %s\n", GPSFilePath.c_str());
        ROS_BREAK();
        return 0;
        }
        double lat, lon, alt, roll, pitch, yaw;
        double vn, ve, vf, vl, vu;
        double ax, ay, az, af, al, au;
        double wx, wy, wz, wf, wl, wu;
        double pos_accuracy, vel_accuracy;
        double navstat, numsats;
        double velmode, orimode;

        fscanf(GPSFile, "%lf %lf %lf %lf %lf %lf ", &lat, &lon, &alt, &roll, &pitch, &yaw);
        //printf("lat:%lf lon:%lf alt:%lf roll:%lf pitch:%lf yaw:%lf \n",  lat, lon, alt, roll, pitch, yaw);
        fscanf(GPSFile, "%lf %lf %lf %lf %lf ", &vn, &ve, &vf, &vl, &vu);
        //printf("vn:%lf ve:%lf vf:%lf vl:%lf vu:%lf \n",  vn, ve, vf, vl, vu);
        fscanf(GPSFile, "%lf %lf %lf %lf %lf %lf ", &ax, &ay, &az, &af, &al, &au);
        //printf("ax:%lf ay:%lf az:%lf af:%lf al:%lf au:%lf\n",  ax, ay, az, af, al, au);
        fscanf(GPSFile, "%lf %lf %lf %lf %lf %lf ", &wx, &wy, &wz, &wf, &wl, &wu);
        //printf("wx:%lf wy:%lf wz:%lf wf:%lf wl:%lf wu:%lf\n",  wx, wy, wz, wf, wl, wu);
        fscanf(GPSFile, "%lf %lf %lf %lf %lf %lf ", &pos_accuracy, &vel_accuracy, &navstat, &numsats, &velmode,
        &orimode);
        //printf("pos_accuracy:%lf vel_accuracy:%lf navstat:%lf numsats:%lf velmode:%lf orimode:%lf\n",
        //	    pos_accuracy, vel_accuracy, navstat, numsats, velmode, orimode);

        std::fclose(GPSFile);

        Vector3d linearAcceleration(ax, ay, az);
        Vector3d angularVelocity(wx, wy, wz);
        sensor_msgs::Imu msgImu;
        msgImu.header.frame_id = "body";
        msgImu.header.stamp = ros::Time(itPairTimeType->first);
        msgImu.linear_acceleration.x = ax;
        msgImu.linear_acceleration.y = ay;
        msgImu.linear_acceleration.z = az;
        msgImu.angular_velocity.x = wx;
        msgImu.angular_velocity.y = wy;
        msgImu.angular_velocity.z = wz;

        pubImu.publish(msgImu);
        //ROS_INFO("Input IMU, time:%f, seq:%d", itPairTimeType->first, imu_gnssIdx);
        //                fprintf(tmp_outFile, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
        //                        itPairTimeType->first, ax, ay, az, af, al, au, wx, wy, wz, wf, wl, wu);
        //                    estimator.inputIMU(itPairTimeType->first, linearAcceleration, angularVelocity);

        if ((imu_gnssIdx % gnss_ex_div == 0) && lat>0)
        {
        sensor_msgs::NavSatFix gps_position;
        gps_position.header.frame_id = "gnss";
        gps_position.header.stamp = ros::Time(itPairTimeType->first);
        gps_position.status.status = navstat;
        gps_position.status.service = numsats;
        gps_position.latitude = lat;
        gps_position.longitude = lon;
        gps_position.altitude = alt;
        gps_position.position_covariance[0] = pos_accuracy;
        gps_position.position_covariance[4] = pos_accuracy;
        gps_position.position_covariance[8] = pos_accuracy;

        pubOrignGnss.publish(gps_position);
        //ROS_INFO("Input GNSS, time:%f, seq:%d", itPairTimeType->first, imu_gnssIdx);
        //                        estimator.inputGNSS(itPairTimeType->first, gps_position);
        }
        }
        //sleep function, set this value to control the publishment frequency
         std::this_thread::sleep_for(std::chrono::milliseconds(SENSOR_SLEEP_TIME));
        }
        //           if (itPairTimeType->second == CAM)
        //          {
        //              Eigen::Matrix<double, 4, 4> pose;
        //              estimator.getPoseInWorldFrame(pose);
        //                if (outFile != NULL)
        //                    fprintf(outFile, "%f %f %f %f %f %f %f %f %f %f %f %f\n",
        //                            pose(0, 0), pose(0, 1), pose(0, 2), pose(0, 3),
        //                            pose(1, 0), pose(1, 1), pose(1, 2), pose(1, 3),
        //                            pose(2, 0), pose(2, 1), pose(2, 2), pose(2, 3));
        //                Eigen::Matrix<double, 4, 4> Tic1;
        //                estimator.getEstimatedExPara(0,Tic1);
        //                std::cout<< "-------\n" << Tic1 <<std::endl;
        //                Eigen::Matrix<double, 4, 4> Tic2;
        //                estimator.getEstimatedExPara(1,Tic2);
        //                std::cout<< "-------\n"<< Tic2 <<std::endl;
        //cv::imshow("leftImage", imLeft);
        //cv::imshow("rightImage", imRight);
        //cv::waitKey(2);
        //           }

        }
        else
        break;

    }
    //    if(outFile != NULL)
    //        fclose (outFile);
    //    ROS_INFO("####### SYSTEM END ################");
    ////    if(tmp_outFile != NULL)
    ////        fclose (tmp_outFile);

ros::spin();

return 0;
}

