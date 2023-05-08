#include "GxIAPI.h"
#include "DxImageProc.h"
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <pthread.h>
#include <queue>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <mavros_msgs/CamIMUStamp.h>
#include <mavros_msgs/CommandTriggerControl.h>
#include <mavros_msgs/CommandTriggerInterval.h>
#include <jetson-utils/cudaMappedMemory.h>
#include <jetson-utils/cudaColorspace.h>
#include <jetson-utils/imageIO.h>
#include <nppi.h>
#include <cuda_runtime_api.h>
#include <yaml-cpp/yaml.h>
#include <string.h>
#include <chrono>
#include <fstream>

#include <vpi/OpenCVInterop.hpp>
#include <vpi/Image.h>
#include <vpi/Status.h>
#include <vpi/Stream.h>
#include <vpi/algo/ConvertImageFormat.h>
#include <vpi/algo/Rescale.h>
#include <vpi/algo/StereoDisparity.h>
#include <mutex>

#include <NvBufSurface.h>
#include <nvbuf_utils.h>
#include <NvBuffer.h>
// #include <glog/logging.h>
using namespace std;
using namespace chrono;
#define CHECK_STATUS(STMT)                                    \
    do                                                        \
    {                                                         \
        VPIStatus status = (STMT);                            \
        if (status != VPI_SUCCESS)                            \
        {                                                     \
            char buffer[VPI_MAX_STATUS_MESSAGE_LENGTH];       \
            vpiGetLastStatusMessage(buffer, sizeof(buffer));  \
            std::ostringstream ss;                            \
            ss << vpiStatusGetName(status) << ": " << buffer; \
            throw std::runtime_error(ss.str());               \
        }                                                     \
    } while (0);

// 用于同步的大恒相机驱动
class Gxdrive
{
private:                                                      // 参数
    double left_intrinsics[9], right_intrinsics[9], cam_r[9]; // 相机内参
    double right_distortion[4], left_distortion[4];           // 相机畸变参数矩阵
    double left_p[12], right_p[12];                           //
    // 大恒相机设置参数:曝光时间，rgb白平衡，增益，pwm周期，pwm高电平，pwm占空比
    double exposure_t, bal_r, bal_g, bal_b, gain, pwm_period, pwm_on, pwm_duty;
    int control_mode, frequency;         // 控制模式，频率
    std::string camera_style;            // 相机类型
    uint32_t height, width;              // 图像的长宽
    int sync_frame;                      // 初始化帧数
    uint64_t frame = 0;                  // 帧数
    uint64_t last_gframe_time = 0;       // 上一帧fpga时间戳
    uint64_t gframe_dura = 0;            // fpga帧间时间
    bool dropframe = false;              // is drop frame
    double pwm_period_s;                 // pwm_period 以s为单位
    GX_FRAME_DATA g_frame_data = {0};    ///< 采集图像参数
    ros::NodeHandle n;                   // ros handle
    ros::ServiceClient trival_client;    // 控制飞控pwm参数
    ros::ServiceClient tricon_client;    // 控制飞控pwm开关
    ros::Publisher pub_img_left;         // 左图rgb
    ros::Publisher pub_img_right;        // 右图rgb
    ros::Publisher pub_img_left_gray;    // 左图gray
    ros::Publisher pub_img_right_gray;   // 右图gray
    ros::Publisher pub_stereo_disparity; // 右图gray

    ros::Subscriber sub_time;                    // 相机图像时间戳订阅
    ros::Time current_time = ros::Time(1, 0);    // 传输来的时间戳
    ros::Time time_image = ros::Time(0, 0);      // 赋给图片的时间戳
    ros::Time last_time_image = ros::Time(0, 0); // 上一帧赋给图片的时间戳

    mavros_msgs::CommandTriggerInterval trigger_interval; // 控制飞控的pwm频率信息
    mavros_msgs::CommandTriggerControl trigger_control;   // 控制飞控的pwm开关信息

    u_char *left_rgb;    // 左图rgb(Gpu)
    u_char *right_rgb;   // 右图rgb(Gpu)
    u_char *left_gray;   // 左图gray(Gpu)
    u_char *right_gray;  // 右图gray(Gpu)
    u_char *left_gamma;  // 左图rgb，gamma矫正(Gpu)
    u_char *right_gamma; // 右图rgb，gamma矫正(Gpu)
    u_char *tmp_color;   // 未分割图像(Gpu)
    uint32_t id_front = 0;
    bool ispre = true;

    sensor_msgs::Image img_left, img_right;              // 用于发布的ros彩色图像
    sensor_msgs::Image img_left_gray, img_right_gray;    // 用于发布的ros灰度图像
    sensor_msgs::Image stereo_disparity, confidence_map; // 深度图:视差图与置信图

    VPIImageData vpiData_left = {};
    VPIImageData vpiData_right = {};
    VPIImage vpiimage_left = NULL;  // 左图vpiimage
    VPIImage vpiimage_right = NULL; // 右图vpiimage
    VPIStream stream = NULL;
    VPIConvertImageFormatParams cvtParams;
    VPIImage stereoLeft = NULL;
    VPIImage stereoRight = NULL;
    VPIImage disparity = NULL;
    VPIImage confidenceMap = NULL;
    VPIImage confidenceMap_u8 = NULL;
    VPIPayload stereo = NULL;
    VPIConvertImageFormatParams convParams;
    VPIConvertImageFormatParams convParams_U8;
    VPIImageData vpidata16 = {};
    VPIStereoDisparityEstimatorParams params;

public:
    std::mutex lock;
    GX_DEV_HANDLE g_device = NULL;  ///< 设备句柄
    pthread_t g_acquire_thread = 0; ///< 采集线程ID
    pthread_t depth_thread = 0;     ///< 深度图线程ID
    // 读取yaml文件进行参数初始化
    void yaml_read(std::string file_path);
    // 数据结构初始化
    void Init();
    // 获取图像size
    NppiSize getImageSize() { return {(int)width, (int)height}; };
    // 启动相机
    GX_STATUS OpenGx();
    /*
    \brief  获取错误信息描述
    \param  emErrorStatus  错误码
    \return void
    */
    void GetErrorString(GX_STATUS error_status);
    /*
    \brief 释放资源
    \return void
    */
    int UnPreForImage();
    /*
    \brief 采集线程函数
    \param pParam 线程传入参数
    \return void*
    */
    void *ProcGetImage(void *pParam);
    // 订阅时间戳的回调函数
    void Time_callback(const mavros_msgs::CamIMUStampConstPtr &cam_stamp);
    // log 信息打印
    void initLog(const char *logfile);
    // 深度图
    void depth_map();
};