#include "../include/GxIAPI.h"
#include "../include/DxImageProc.h"
#include <stdio.h>
#include <tbb/concurrent_queue.h>
#include <pthread.h>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <mavros_msgs/CamIMUStamp.h>
#include <mavros_msgs/CommandTriggerControl.h>
#include <mavros_msgs/CommandTriggerInterval.h>
#include <yaml-cpp/yaml.h>
#include <string.h>
#include <chrono>
using namespace std;
using namespace cv;
using namespace cv::dnn;

#define MEMORY_ALLOT_ERROR -1

uchar g_GammaLUT[256];            // 全局数组：包含256个元素的gamma校正查找表
GX_DEV_HANDLE g_device = NULL;    ///< 设备句柄
GX_FRAME_DATA g_frame_data = {0}; ///< 采集图像参数
pthread_t g_acquire_thread = 0;   ///< 采集线程ID
ros::Publisher pub_img_left;
ros::Publisher pub_img_right;
ros::Publisher pub_img_left_gray;
ros::Publisher pub_img_right_gray;
ros::Publisher pub_cam_left_info;
ros::Publisher pub_cam_right_info;

std::mutex mtx_photo; // 定义互斥锁
ros::Subscriber sub_time;
ros::Time last_tmp_time;
ros::Duration last_tmp_inter, duration_min(1, 0);
std::queue<uint32_t> id_q;
std::queue<ros::Time> time_q;
std::queue<sensor_msgs::Image> photo_left_q, photo_right_q;
// tbb::concurrent_bounded_queue<sensor_msgs::Image> photo_left_q, photo_right_q;
uint32_t id_image = 0, id_image_ = 0;
uint32_t id_front = 0;
bool isfirst = true;
bool ispre = true;

double left_intrinsics[9], right_intrinsics[9], cam_r[9];
double right_distortion[4], left_distortion[4];
double left_p[12], right_p[12];
double exposure_t, gamma_ratio, bal_r, bal_g, bal_b, gain, pwm_period, pwm_on, pwm_duty;
int control_mode, frequency, resolution;
std::string camera_style;
uint32_t height, width;
sensor_msgs::CameraInfo cam_left, cam_right;

sensor_msgs::Image img_left, img_right;
sensor_msgs::Image img_left_gray, img_right_gray;
sensor_msgs::Image *img_left_gray_p;
sensor_msgs::Image *img_right_gray_p;

ros::ServiceClient tricon_client;
mavros_msgs::CommandTriggerControl trigger_control;

uint8_t *m_rgb_image = NULL; // 增加的内容

// 获取图像大小并申请图像数据空间
int PreForImage();

// 释放资源
int UnPreForImage();

// 采集线程函数
void *ProcGetImage(void *param);

// 获取错误信息描述
void GetErrorString(GX_STATUS error_status);

inline void spilt(uchar *img_1, uchar *img_2, const uchar *imgraw, size_t nWidth, size_t nHeight);
void resize_spilt(uchar *img_1, uchar *img_2, const uchar *imgraw, size_t nWidth, size_t nHeight);
void rbg24togray(uchar *imgrgb, uchar *imggray, size_t nWidth, size_t nHeight);
void GammaCorrectiom(uchar *src, int iWidth, int iHeight, uchar *Dst);
void BuildTable(float fPrecompensation);
void Time_callback(const mavros_msgs::CamIMUStampConstPtr &cam_stamp)
{
    id_q.push(cam_stamp->frame_seq_id);
    time_q.push(cam_stamp->frame_stamp);
    if (isfirst)
    {
        id_q.pop();
        time_q.pop();
        isfirst = false;
    }
    ROS_DEBUG("Image size:%ld,Time size:%ld,ID size:%ld", photo_left_q.size(), time_q.size(), id_q.size());
    if (!photo_left_q.empty() && !photo_right_q.empty())
    {

        if (id_front != cam_stamp->frame_seq_id - 1)
        {
            ROS_WARN("drop frame! id_front:%d,now:%d", id_front, cam_stamp->frame_seq_id);
        }
        if ((last_tmp_inter - (time_q.front() - last_tmp_time)).toSec() > 0.001 || (last_tmp_inter - (time_q.front() - last_tmp_time)).toSec() < -0.001)
        {
            ROS_ERROR("id:%d,inter too large:%lf,%lf", cam_stamp->frame_seq_id, last_tmp_inter.toSec(), (time_q.front() - last_tmp_time).toSec());
            ROS_ERROR("timestamp:%lf", time_q.front().toSec());
        }
        last_tmp_inter = time_q.front() - last_tmp_time;
        if (last_tmp_inter.toSec() < -0.001)
        {
            ROS_ERROR("time error,the past ahead of the present");
            ROS_ERROR("current:%lf,last:%lf", time_q.front().toSec(), last_tmp_time.toSec());
        }
        if (duration_min > last_tmp_inter && last_tmp_inter.toSec() > 0.001)
        {
            duration_min = last_tmp_inter;
        }
        ROS_DEBUG("duration-ming:%lf", duration_min.toSec());
        last_tmp_time = time_q.front(); // 当前的时间戳

        if (!time_q.empty())
        {
            time_q.pop();
        }
        if (id_q.size() > 1024)
        {
            ROS_ERROR("exit!The length of queue exceeds 1024!");
            exit(0);
        }
        else if (id_q.empty())
        {
            ROS_WARN("id is empty,img is no stamp!");
        }
        else
        {
            ROS_DEBUG("The image_id:%u", id_q.front());
            if (id_image != id_q.front() - 1)
            {
                ROS_WARN("not sync,image_id:%u,id:%u", id_image, id_q.front() - 1);
            }
            id_q.pop();
        }
        mtx_photo.lock();
        while (!photo_left_q.empty())
        {

            if (photo_left_q.size() > 1)
            {
                ROS_WARN("id is %d,Image size is %ld", cam_stamp->frame_seq_id, photo_left_q.size());
                ROS_ERROR("timestamp:%lf", last_tmp_time.toSec());
            }
            img_left_gray_p = &photo_left_q.front();
            img_right_gray_p = &photo_right_q.front();
            // 漏发时间戳
            if (id_q.empty())
            {
                // 左图灰度图
                img_left_gray_p->header.stamp = last_tmp_time - ros::Duration(0, exposure_t * 500) - duration_min * (photo_left_q.size() - 1);
                img_left_gray_p->header.seq = id_image_;
                // 右图灰度图
                img_right_gray_p->header.stamp = last_tmp_time - ros::Duration(0, exposure_t * 500) - duration_min * (photo_right_q.size() - 1);
                img_right_gray_p->header.seq = id_image_;
                // cam_left_info
                cam_left.header.stamp = last_tmp_time - ros::Duration(0, exposure_t * 500) - duration_min * (photo_right_q.size() - 1);
                cam_left.header.seq = id_image_;
                // cam_right_info
                cam_right.header.stamp = last_tmp_time - ros::Duration(0, exposure_t * 500) - duration_min * (photo_right_q.size() - 1);
                cam_right.header.seq = id_image_;
            }
            // 图像处理时间过长导致队列堆积
            else
            {
                id_image++;
                img_left_gray_p->header.stamp = last_tmp_time - ros::Duration(0, exposure_t * 500);
                img_left_gray_p->header.seq = id_image_;
                // 右图灰度图
                img_right_gray_p->header.stamp = last_tmp_time - ros::Duration(0, exposure_t * 500);
                img_right_gray_p->header.seq = id_image_;
                // cam_left_info
                cam_left.header.stamp = last_tmp_time - ros::Duration(0, exposure_t * 500);
                cam_left.header.seq = id_image_;
                // cam_right_info
                cam_right.header.stamp = last_tmp_time - ros::Duration(0, exposure_t * 500);
                cam_right.header.seq = id_image_;
                last_tmp_time = time_q.front(); // 当前的时间戳
                id_q.pop();
                time_q.pop();
            }
            pub_img_left_gray.publish(*img_right_gray_p);
            pub_img_right_gray.publish(*img_left_gray_p);
            pub_cam_left_info.publish(cam_left);
            pub_cam_right_info.publish(cam_right);
            img_left_gray_p = NULL;
            img_right_gray_p = NULL;
            photo_left_q.pop();
            photo_right_q.pop();
            id_image_++;
        }
        id_front = cam_stamp->frame_seq_id;
        id_image++;
        mtx_photo.unlock();
    }
    else
    {
        ROS_WARN("The image quene is empty");
        ROS_WARN("the id length is %ld", id_q.size());
    }
}
void yaml_read(std::string file_path)
{

    YAML::Node yaml_node = YAML::LoadFile(file_path);
    camera_style = yaml_node["Camera_style"].as<std::string>();
    control_mode = yaml_node["Control_mode"].as<int>();
    frequency = yaml_node["Frequency"].as<int>();
    resolution = yaml_node["Resolution"].as<int>();
    gamma_ratio = yaml_node["Gamma"].as<double>();
    gain = yaml_node["Gain"].as<double>();
    pwm_period = 1000 / yaml_node["Pwm_fre"].as<double>();
    pwm_duty = yaml_node["Duty"].as<double>();
    pwm_on = pwm_duty * pwm_period;
    exposure_t = yaml_node["Exposure_T"].as<double>();
    bal_r = yaml_node["Balance_ratio"]["r"].as<double>();
    bal_g = yaml_node["Balance_ratio"]["g"].as<double>();
    bal_b = yaml_node["Balance_ratio"]["b"].as<double>();
    for (int i = 0; i < 9; i++)
    {
        left_intrinsics[i] = yaml_node["Left_cam"]["intrinsics"][i].as<double>();
        right_intrinsics[i] = yaml_node["Right_cam"]["intrinsics"][i].as<double>();
        cam_r[i] = yaml_node["Cam_r"][i].as<double>();
    }
    for (int i = 0; i < 4; i++)
    {
        left_distortion[i] = yaml_node["Left_cam"]["distortion"][i].as<double>();
        right_distortion[i] = yaml_node["Right_cam"]["distortion"][i].as<double>();
    }
    for (int i = 0; i < 12; i++)
    {
        left_p[i] = yaml_node["Left_cam"]["Cam_p"][i].as<double>();
        right_p[i] = yaml_node["Right_cam"]["Cam_p"][i].as<double>();
    }
    height = (resolution == 0) ? 512 : 1024;
    width = (resolution == 0) ? 640 : 1280;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cam");
    ros::NodeHandle n;
    // photo_left_q.set_capacity(10);
    // photo_right_q.set_capacity(10);
    pub_img_left = n.advertise<sensor_msgs::Image>("/cam/left/bgr", 1000);
    pub_img_right = n.advertise<sensor_msgs::Image>("/cam/right/bgr", 1000);
    pub_img_left_gray = n.advertise<sensor_msgs::Image>("/cam/left/gray", 1000);
    pub_img_right_gray = n.advertise<sensor_msgs::Image>("/cam/right/gray", 1000);
    sub_time = n.subscribe<mavros_msgs::CamIMUStamp>("/mavros/sync/cam_imu_stamp", 100, Time_callback);
    pub_cam_left_info = n.advertise<sensor_msgs::CameraInfo>("/cam/left/info", 1000);
    pub_cam_right_info = n.advertise<sensor_msgs::CameraInfo>("/cam/right/info", 1000);
    tricon_client = n.serviceClient<mavros_msgs::CommandTriggerControl>("/mavros/cmd/trigger_control");
    ros::ServiceClient trival_client = n.serviceClient<mavros_msgs::CommandTriggerInterval>("/mavros/cmd/trigger_interval");
    // 读取参数
    yaml_read("/home/oem/Vscode/work/cam/camera.yaml");
    mavros_msgs::CommandTriggerInterval trigger_interval;
    trigger_interval.request.cycle_time = pwm_period;
    trigger_interval.request.integration_time = pwm_on;
    trival_client.call(trigger_interval);
    trigger_control.request.sequence_reset = true;
    trigger_control.request.trigger_enable = false;
    trigger_control.request.trigger_pause = false;
    tricon_client.call(trigger_control);
    if (control_mode == 1)
    {
        trigger_control.request.sequence_reset = false;
        trigger_control.request.trigger_enable = true;
        trigger_control.request.trigger_pause = false;
    }
    BuildTable(1.0 / gamma_ratio);
    printf("Initializing......");
    printf("\n\n");

    // camera left info
    cam_left.header.frame_id = "cam0";
    cam_left.height = height;
    cam_left.width = width;
    std::copy(std::begin(left_intrinsics), std::end(left_intrinsics), std::begin(cam_left.K));
    cam_left.D.reserve(4);
    std::copy(std::begin(left_distortion), std::end(left_distortion), std::begin(cam_left.D));
    std::copy(std::begin(left_p), std::end(left_p), std::begin(cam_left.P));
    std::copy(std::begin(cam_r), std::end(cam_r), std::begin(cam_left.R));
    cam_left.roi.height = height;
    cam_left.roi.width = width;
    cam_left.roi.x_offset = 0;
    cam_left.roi.y_offset = 0;
    // camera right info
    cam_right.header.frame_id = "cam1";
    cam_right.height = height;
    cam_right.width = width;
    std::copy(std::begin(right_intrinsics), std::end(right_intrinsics), std::begin(cam_right.K));
    cam_right.D.reserve(4);
    std::copy(std::begin(right_distortion), std::end(right_distortion), std::begin(cam_right.D));
    std::copy(std::begin(cam_r), std::end(cam_r), std::begin(cam_right.R));
    std::copy(std::begin(right_p), std::end(right_p), std::begin(cam_right.P));
    cam_right.roi.height = height;
    cam_right.roi.width = width;
    cam_right.roi.x_offset = 0;
    cam_right.roi.y_offset = 0;
    // 左图
    img_left.header.frame_id = "cam0";
    img_left.height = height;
    img_left.width = width;
    img_left.encoding = "bgr8";
    img_left.is_bigendian = false;
    img_left.step = width * 3;
    img_left.data.resize(img_left.step * height);
    // 右图
    img_right.header.frame_id = "cam1";
    img_right.height = height;
    img_right.width = width;
    img_right.encoding = "bgr8";
    img_right.is_bigendian = false;
    img_right.step = width * 3;
    img_right.data.resize(img_right.step * height);
    // 左图灰度
    img_left_gray.header.frame_id = "cam0";
    img_left_gray.height = height;
    img_left_gray.width = width;
    img_left_gray.encoding = "mono8";
    img_left_gray.is_bigendian = false;
    img_left_gray.step = width;
    img_left_gray.data.resize(img_left_gray.step * height);
    // 右图灰度
    img_right_gray.header.frame_id = "cam1";
    img_right_gray.height = height;
    img_right_gray.width = width;
    img_right_gray.encoding = "mono8";
    img_right_gray.is_bigendian = false;
    img_right_gray.step = width;
    img_right_gray.data.resize(img_right_gray.step * height);
    // usleep(1000000);

    // API接口函数返回值
    GX_STATUS status = GX_STATUS_SUCCESS;

    uint32_t device_num = 0;
    uint32_t ret = 0;
    GX_OPEN_PARAM open_param;

    // 初始化设备打开参数，默认打开序号为1的设备
    open_param.accessMode = GX_ACCESS_CONTROL;
    open_param.openMode = GX_OPEN_INDEX;
    open_param.pszContent = (char *)"1";

    // 初始化库
    status = GXInitLib();
    // 获取枚举设备个数
    status = GXUpdateDeviceList(&device_num, 1000);

    if (device_num <= 0)
    {
        printf("<No device>\n");
        status = GXCloseLib();
        return 0;
    }
    else
    {
        // 默认打开第1个设备
        status = GXOpenDevice(&open_param, &g_device);
        if (status == GX_STATUS_SUCCESS)
        {
            printf("<Open device success>\n");
        }
        else
        {
            printf("<Open device fail>\n");
            status = GXCloseLib();
            return 0;
        }
    }
    status = GX_STATUS_SUCCESS;

    // 设置最小增益值
    status = GXSetEnum(g_device, GX_ENUM_GAIN_SELECTOR,
                       GX_GAIN_SELECTOR_ALL);
    status = GXSetFloat(g_device, GX_FLOAT_GAIN, gain);

    // 设置白平衡系数
    status = GXSetEnum(g_device, GX_ENUM_BALANCE_RATIO_SELECTOR,
                       GX_BALANCE_RATIO_SELECTOR_BLUE);
    status = GXSetFloat(g_device, GX_FLOAT_BALANCE_RATIO,
                        bal_b);

    status = GXSetEnum(g_device, GX_ENUM_BALANCE_RATIO_SELECTOR,
                       GX_BALANCE_RATIO_SELECTOR_GREEN);
    status = GXSetFloat(g_device, GX_FLOAT_BALANCE_RATIO,
                        bal_g);

    status = GXSetEnum(g_device, GX_ENUM_BALANCE_RATIO_SELECTOR,
                       GX_BALANCE_RATIO_SELECTOR_RED);
    status = GXSetFloat(g_device, GX_FLOAT_BALANCE_RATIO,
                        bal_r);
    // 设置采集模式为连续采集
    status = GXSetEnum(g_device, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
    // 引脚选择为 Line2
    status = GXSetEnum(g_device, GX_ENUM_LINE_SELECTOR,
                       GX_ENUM_LINE_SELECTOR_LINE2);
    // 设置引脚方向为输入
    status = GXSetEnum(g_device, GX_ENUM_LINE_MODE,
                       GX_ENUM_LINE_MODE_INPUT);
    // 设置触发开关为ON
    status = GXSetEnum(g_device, GX_ENUM_TRIGGER_MODE, control_mode);
    // 设置触发激活方式为上升沿
    status = GXSetEnum(g_device, GX_ENUM_TRIGGER_ACTIVATION,
                       GX_TRIGGER_ACTIVATION_RISINGEDGE);
    // 设置上升沿滤波最小值
    GX_FLOAT_RANGE raisingRange;
    status = GXGetFloatRange(g_device, GX_FLOAT_TRIGGER_FILTER_RAISING,
                             &raisingRange);
    status = GXSetFloat(g_device, GX_FLOAT_TRIGGER_FILTER_RAISING,
                        raisingRange.dMax);
    // 设置触发源
    status = GXSetEnum(g_device, GX_ENUM_TRIGGER_SOURCE, GX_TRIGGER_SOURCE_LINE2);
    // 设置曝光模式
    status = GXSetEnum(g_device, GX_ENUM_EXPOSURE_MODE,
                       GX_EXPOSURE_MODE_TIMED);
    // 设置曝光时间
    status = GXSetFloat(g_device, GX_FLOAT_EXPOSURE_TIME,
                        exposure_t);
    // 举例引脚选择为 Line3
    status = GXSetEnum(g_device, GX_ENUM_LINE_SELECTOR,
                       GX_ENUM_LINE_SELECTOR_LINE3);
    // 设置引脚方向为输出
    status = GXSetEnum(g_device, GX_ENUM_LINE_MODE,
                       GX_ENUM_LINE_MODE_OUTPUT);
    // 可选操作引脚电平反转
    status = GXSetBool(g_device, GX_BOOL_LINE_INVERTER, true);
    // 如果设置输出源为闪光灯,则如下代码
    status = GXSetEnum(g_device, GX_ENUM_LINE_SOURCE,
                       GX_ENUM_LINE_SOURCE_STROBE);
    // 设置采集帧率
    if (control_mode == 0)
    {
        status = GXSetEnum(g_device, GX_ENUM_ACQUISITION_FRAME_RATE_MODE,
                           GX_ACQUISITION_FRAME_RATE_MODE_ON);
        status = GXSetFloat(g_device, GX_FLOAT_ACQUISITION_FRAME_RATE, frequency);
    }
    // 为采集做准备
    ret = PreForImage();
    if (ret != 0)
    {
        printf("<Failed to prepare for acquire image>\n");
        status = GXCloseDevice(g_device);
        if (g_device != NULL)
        {
            g_device = NULL;
        }
        status = GXCloseLib();
        return 0;
    }
    // 启动接收线程
    m_rgb_image = new uint8_t[2560 * 1024 * 3];
    ret = pthread_create(&g_acquire_thread, 0, ProcGetImage, 0);
    if (ret != 0)
    {
        printf("<Failed to create the collection thread>\n");
        status = GXCloseDevice(g_device);
        if (g_device != NULL)
        {
            g_device = NULL;
        }
        status = GXCloseLib();
        return 0;
    }
    ros::spin();
    // 为停止采集做准备
    ret = UnPreForImage();
    // 关闭设备
    status = GXCloseDevice(g_device);

    // 释放库
    status = GXCloseLib();
    return 0;
}

//-------------------------------------------------
/**
\brief 获取图像大小并申请图像数据空间
\return void
*/
//-------------------------------------------------
int PreForImage()
{
    GX_STATUS status = GX_STATUS_SUCCESS;
    int64_t payload_size;

    status = GXGetInt(g_device, GX_INT_PAYLOAD_SIZE, &payload_size);
    if (status != GX_STATUS_SUCCESS)
    {
        GetErrorString(status);
        std::cout << "haihiahf" << std::endl;
        return status;
    }

    g_frame_data.pImgBuf = malloc(payload_size);
    if (g_frame_data.pImgBuf == NULL)
    {
        printf("<Failed to allot memory>\n");
        return MEMORY_ALLOT_ERROR;
    }

    return 0;
}

//-------------------------------------------------
/**
\brief 释放资源
\return void
*/
//-------------------------------------------------
int UnPreForImage()
{
    GX_STATUS status = GX_STATUS_SUCCESS;
    uint32_t ret = 0;
    ROS_INFO("exit Image!");
    // 发送停采命令
    status = GXSendCommand(g_device, GX_COMMAND_ACQUISITION_STOP);
    if (status != GX_STATUS_SUCCESS)
    {
        GetErrorString(status);
        return status;
    }
    ret = pthread_join(g_acquire_thread, NULL);
    if (ret != 0)
    {
        printf("<Failed to release resources>\n");
        return ret;
    }

    // 释放buffer
    if (g_frame_data.pImgBuf != NULL)
    {
        free(g_frame_data.pImgBuf);
        g_frame_data.pImgBuf = NULL;
    }

    return 0;
}

//-------------------------------------------------
/**
\brief 采集线程函数
\param pParam 线程传入参数
\return void*
*/
//-------------------------------------------------
void *ProcGetImage(void *pParam)
{
    std::cout << "Enter acquire!" << std::endl;
    GX_STATUS status = GX_STATUS_SUCCESS;
    // 发送开采命令
    status = GXSendCommand(g_device, GX_COMMAND_ACQUISITION_START);
    if (status != GX_STATUS_SUCCESS)
    {
        GetErrorString(status);
    }
    while (ros::ok())
    {
        if (g_frame_data.pImgBuf == NULL)
        {
            ROS_WARN("Image is NULL!");
            continue;
        }
        status = GXGetImage(g_device, &g_frame_data, 100);
        if (status == GX_STATUS_SUCCESS)
        {
            if (g_frame_data.nStatus == 0)
            {
                // 修改这个的高和宽是否可以直接改变图像大小？
                DxRaw8toRGB24(g_frame_data.pImgBuf, m_rgb_image, g_frame_data.nWidth, g_frame_data.nHeight, RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(BAYERBG), false);
                // memcpy(reinterpret_cast<uchar *>(&img.data[0]), m_rgb_image, g_frame_data.nWidth * g_frame_data.nHeight * 3);
                resize_spilt(&img_right.data[0], &img_left.data[0], m_rgb_image, g_frame_data.nWidth, g_frame_data.nHeight);
                rbg24togray(reinterpret_cast<uchar *>(&img_right.data[0]), reinterpret_cast<uchar *>(&img_right_gray.data[0]), width, height);
                rbg24togray(reinterpret_cast<uchar *>(&img_left.data[0]), reinterpret_cast<uchar *>(&img_left_gray.data[0]), width, height);
                GammaCorrectiom(reinterpret_cast<uchar *>(&img_right_gray.data[0]), width, height, reinterpret_cast<uchar *>(&img_right_gray.data[0]));
                GammaCorrectiom(reinterpret_cast<uchar *>(&img_left_gray.data[0]), width, height, reinterpret_cast<uchar *>(&img_left_gray.data[0]));

                mtx_photo.lock();
                photo_left_q.push(img_left_gray);
                photo_right_q.push(img_right_gray);
                mtx_photo.unlock();
                // 左图
                img_left.header.stamp = last_tmp_time;
                // 右图
                img_right.header.stamp = last_tmp_time;
                pub_img_left.publish(img_right);
                pub_img_right.publish(img_left);
                while (photo_left_q.size() > 2)
                {
                }
            }
        }
        else
        {
            ROS_WARN("cannot get image");
            if (control_mode == 1)
            {
                if (!ispre)
                {
                    ros::shutdown();
                    break;
                }
                ispre = false;
                tricon_client.call(trigger_control);
            }
        }
    }
    return 0;
}

//----------------------------------------------------------------------------------
/**
\brief  获取错误信息描述
\param  emErrorStatus  错误码

\return void
*/
//----------------------------------------------------------------------------------
void GetErrorString(GX_STATUS error_status)
{
    char *error_info = NULL;
    size_t size = 0;
    GX_STATUS status = GX_STATUS_SUCCESS;

    // 获取错误描述信息长度
    status = GXGetLastError(&error_status, NULL, &size);
    if (status != GX_STATUS_SUCCESS)
    {
        GetErrorString(status);
        return;
    }

    error_info = new char[size];
    if (error_info == NULL)
    {
        printf("<Failed to allocate memory>\n");
        return;
    }

    // 获取错误信息描述
    status = GXGetLastError(&error_status, error_info, &size);
    if (status != GX_STATUS_SUCCESS)
    {
        printf("<GXGetLastError call fail>\n");
    }
    else
    {
        printf("%s\n", (char *)error_info);
    }

    // 释放资源
    if (error_info != NULL)
    {
        delete[] error_info;
        error_info = NULL;
    }
}

inline void spilt(uchar *img_1, uchar *img_2, const uchar *imgraw, size_t nWidth, size_t nHeight)
{
    if (NULL == img_1 || NULL == img_1 || NULL == imgraw)
    {
        return;
    }
    else
    {
        size_t height = 0;
        while (height < nHeight)
        {
            memcpy(img_1 + height * nWidth / 2 * 3, imgraw + height * nWidth * 3, 3 * nWidth / 2);
            memcpy(img_2 + height * nWidth / 2 * 3, imgraw + +height * nWidth * 3 + 3 * nWidth / 2, 3 * nWidth / 2);
            height++;
        }
    }
}

void resize_spilt(uint8_t *img_1, uint8_t *img_2, const uint8_t *imgraw, size_t nWidth, size_t nHeight)
{
    if (NULL == img_1 || NULL == img_2 || NULL == imgraw)
    {
        return;
    }

    else
    {
        int height = 0;
        int width = 0;
        int width_2 = nWidth >> 1;
        int width_2_3 = 3 * width_2;
        while (height < nHeight)
        {
            while (width < width_2)
            {
                // 像素点rgb复制
                *img_1 = *imgraw;
                *img_2 = *(imgraw + width_2_3);
                ++imgraw;
                ++img_1;
                ++img_2;

                *img_1 = *imgraw;
                *img_2 = *(imgraw + width_2_3);
                ++imgraw;
                ++img_1;
                ++img_2;

                *img_1 = *imgraw;
                *img_2 = *(imgraw + width_2_3);
                ++imgraw;
                ++img_1;
                ++img_2;
                // cout << (int)*(img_1-1) << endl;
                // 跳过一个像素点
                imgraw += 3;
                width += 2;
            }
            width = 0;
            // 跳过1.5行
            imgraw += 9 * width_2;
            height += 2;
        }
        height = 0;
    }
}

void rbg24togray(uchar *imgrgb, uchar *imggray, size_t nWidth, size_t nHeight)
{
    uchar *end = imgrgb + nWidth * nHeight * 3;
    for (; imgrgb < end; imgrgb = imgrgb + 3, imggray++)
    {
        *imggray = (*imgrgb * 19595 + *(imgrgb + 1) * 38469 + *(imgrgb + 2) * 7472) >> 16;
    }
}

void BuildTable(float fPrecompensation)
{
    int i;
    float f;
    for (i = 0; i < 256; i++)
    {
        f = (i + 0.5F) / 256;                    // 归一化
        f = (float)pow(f, fPrecompensation);     // 预补偿
        g_GammaLUT[i] = (uchar)(f * 256 - 0.5F); // 反归一化
    }
}

void GammaCorrectiom(uchar *src, int iWidth, int iHeight, uchar *Dst)
{
    int iCols, iRows;
    // 对图像的每个像素进行查找表矫正
    for (iRows = 0; iRows < iHeight; iRows++)
    {
        int pixelsNums = iRows * iWidth;
        for (iCols = 0; iCols < iWidth; iCols++)
        {
            // Dst[iRows*iWidth+iCols]=g_GammaLUT[src[iRows*iWidth+iCols]];
            Dst[pixelsNums + iCols] = g_GammaLUT[src[pixelsNums + iCols]];
        }
    }
}