#include <opencv2/opencv.hpp>

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>

int i = 0;
int t;
cv::Mat left_image_dis, right_image_dis;
cv::Mat mapLx, mapLy, mapRx, mapRy;
cv::Mat Rl, Rr, Pl, Pr, Q;
cv::Ptr<cv::StereoSGBM> sgbm;
int num = 0;
int j;
void image_callback(const sensor_msgs::ImageConstPtr &left_img)
{
    if (num == 0)
    {
        int type_;
        if (j == 0)
        {
            type_ = CV_8UC1;
            std::cout << "gray";
        }
        else
        {
            type_ = CV_8UC3;
            std::cout << "bgr";
        }
        left_image_dis = cv::Mat(left_img->height, left_img->width, type_, const_cast<uchar *>(&left_img->data[0]), left_img->step);
        std::string photo_path = "/share/data/lidar_camera/photo/" + std::to_string(i) + ".bmp";
        cv::imwrite(photo_path, left_image_dis);
        num++;
        std::cout << ",save " << i << ".bmp" << std::endl;
    }
    else if (num == 1)
    {
        ros::shutdown();
    }
}
int main(int argc, char **argv)
{
    i = atoi(argv[1]);
    j = atoi(argv[2]);
    ros::init(argc, argv, "validate");
    ros::NodeHandle nh;
    std::string sub_name;
    if (j == 0)
    {
        sub_name = "/cam/right/gray";
    }
    else
    {
        sub_name = "/cam/right/bgr";
    }
    ros::Subscriber sub_left_image = nh.subscribe(sub_name, 2000, image_callback);

    ros::spin();
    return 0;
}