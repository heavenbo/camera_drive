#include "../include/GxIAPI.h"
#include "../include/DxImageProc.h"
#include <stdio.h>
#include <stdlib.h>
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
#include <jetson-utils/cudaMappedMemory.h>
#include <jetson-utils/cudaColorspace.h>
#include <jetson-utils/imageIO.h>
#include "../include/nvidia.hpp"
#include "/usr/src/jetson_multimedia_api/argus/samples/cudaBayerDemosaic/CudaBayerDemosaicKernel.h"
#include <nppi.h>
#include <cuda_runtime_api.h>
#include <chrono>
using namespace chrono;
using namespace std;
using namespace cv;
using namespace cv::dnn;
int image_a = 0;

// debug

#define MEMORY_ALLOT_ERROR -1
GX_DEV_HANDLE g_device = NULL;    ///< 设备句柄
GX_FRAME_DATA g_frame_data = {0}; ///< 采集图像参数
pthread_t g_acquire_thread = 0;   ///< 采集线程ID
bool g_get_image = false;         ///< 采集线程是否结束的标志：true 运行；false 退出
ros::Publisher pub_img_left;
ros::Publisher pub_img_right;
sensor_msgs::Image img_left, img_right;
uchar *m_rgb_image = NULL; // 增加的内容

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cam");
    ros::NodeHandle n;

    pub_img_left = n.advertise<sensor_msgs::Image>("/image/left", 1);
    pub_img_right = n.advertise<sensor_msgs::Image>("/image/right", 1000);

    usleep(1000000);

    // API接口函数返回值
    GX_STATUS status = GX_STATUS_SUCCESS;

    uint32_t device_num = 0;
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
    status = GXSetFloat(g_device, GX_FLOAT_GAIN, 3);

    // 设置白平衡系数
    status = GXSetEnum(g_device, GX_ENUM_BALANCE_RATIO_SELECTOR,
                       GX_BALANCE_RATIO_SELECTOR_BLUE);
    double B = 1.424984;
    double R = 0.988648;
    status = GXSetFloat(g_device, GX_FLOAT_BALANCE_RATIO,
                        B);

    status = GXSetEnum(g_device, GX_ENUM_BALANCE_RATIO_SELECTOR,
                       GX_BALANCE_RATIO_SELECTOR_GREEN);
    status = GXSetFloat(g_device, GX_FLOAT_BALANCE_RATIO,
                        1.0);

    status = GXSetEnum(g_device, GX_ENUM_BALANCE_RATIO_SELECTOR,
                       GX_BALANCE_RATIO_SELECTOR_RED);
    status = GXSetFloat(g_device, GX_FLOAT_BALANCE_RATIO,
                        R);
    // 设置采集模式为连续采集
    status = GXSetEnum(g_device, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
    // 引脚选择为 Line2
    status = GXSetEnum(g_device, GX_ENUM_LINE_SELECTOR,
                       GX_ENUM_LINE_SELECTOR_LINE2);
    // 设置引脚方向为输入
    status = GXSetEnum(g_device, GX_ENUM_LINE_MODE,
                       GX_ENUM_LINE_MODE_INPUT);
    // 设置触发开关为ON
    status = GXSetEnum(g_device, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
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
    double shutterTime = 30000.0;
    status = GXSetFloat(g_device, GX_FLOAT_EXPOSURE_TIME,
                        shutterTime);
    // 为采集做准备
    int ret = PreForImage();
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
    m_rgb_image = new uchar[2560 * 1024];
    ret = pthread_create(&g_acquire_thread, 0, ProcGetImage, 0);
    // 左图
    img_left.header.frame_id = "cam";
    img_left.height = 1024;
    img_left.width = 2560;
    img_left.encoding = "rgb8";
    img_left.is_bigendian = false;
    img_left.step = 2560 * 3;
    img_left.data.resize(img_left.step * 1024);
    // 右图
    img_right.header.frame_id = "cam";
    img_right.height = 1280;
    img_right.width = 1024;
    img_right.encoding = "mono8";
    img_right.is_bigendian = false;
    img_right.step = 1280;
    img_right.data.resize(img_right.step * 1024);
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
    std::cout << payload_size << std::endl;
    if (status != GX_STATUS_SUCCESS)
    {
        GetErrorString(status);
        std::cout << "haihiahf" << std::endl;
        return status;
    }

    g_frame_data.pImgBuf = malloc(payload_size * 4);
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

    // 发送停采命令
    status = GXSendCommand(g_device, GX_COMMAND_ACQUISITION_STOP);
    if (status != GX_STATUS_SUCCESS)
    {
        GetErrorString(status);
        return status;
    }

    g_get_image = false;
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
    GX_STATUS status = GX_STATUS_SUCCESS;
    // 接收线程启动标志
    g_get_image = true;

    // 发送开采命令
    status = GXSendCommand(g_device, GX_COMMAND_ACQUISITION_START);
    if (status != GX_STATUS_SUCCESS)
    {
        GetErrorString(status);
    }
    NppiSize size;
    size.width = 2560;
    size.height = 1024;
    NppiRect roi;
    roi.x = 0;
    roi.y = 0;
    roi.width = 2560;
    roi.height = 1024;
    int srcStep = 2560;
    int dstStep = 2560 * 3;
    u_char *src;
    u_char *dst;
    u_char *tmp;
    cudaMallocManaged(&src, 2560 * 1024 * 8, cudaMemAttachHost);
    cudaMallocManaged(&tmp, 2560 * 1024 * 8);
    cudaMallocManaged(&dst, 2560 * 1024 * 24);

    while (g_get_image)
    {
        // std::cout << "aaaa" << std::endl;
        if (g_frame_data.pImgBuf == NULL)
        {
            continue;
        }
        status = GXGetImage(g_device, &g_frame_data, 100);
        if (status == GX_STATUS_SUCCESS)
        {
            if (g_frame_data.nStatus == 0)
            {
                // 左图
                auto start = system_clock::now();
                img_left.header.stamp = ros::Time::now();
                // 右图
                img_right.header.stamp = ros::Time::now();

                memcpy(src, g_frame_data.pImgBuf, 2560 * 1024);
                cudaStreamAttachMemAsync(NULL, src, 0, cudaMemAttachGlobal);

                // nppiResize_8u_C1R(src, srcStep, size, roi,
                //                   tmp, 1280, {1280, 512}, {0, 0, 1280, 512}, NPPI_INTER_NN);
                nppiCFAToRGB_8u_C1C3R(src, 2560, {2560, 1024}, {0, 0, 2560, 1024},
                                      dst, 2560 * 3,
                                      NPPI_BAYER_RGGB, NPPI_INTER_UNDEFINED);

                // std::cout << "convert done" << std::endl;
                cudaStreamAttachMemAsync(NULL, src, 0, cudaMemAttachHost);

                cudaStreamSynchronize(NULL);
                memcpy(reinterpret_cast<uchar *>(&img_left.data[0]), dst, 2560 * 1024 * 3);

                auto end = system_clock::now();
                auto duration = duration_cast<microseconds>(end - start);
                cout << "cost: "
                     << double(duration.count()) * microseconds::period::num / microseconds::period::den << "秒" << endl;

                pub_img_left.publish(img_left);

                image_a++;
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

void resize_spilt(uchar *img_1, uchar *img_2, const uchar *imgraw, size_t nWidth, size_t nHeight)
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