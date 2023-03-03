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

using namespace std;
using namespace cv;
using namespace cv::dnn;

#define MEMORY_ALLOT_ERROR -1

uchar g_GammaLUT[256];            // 全局数组：包含256个元素的gamma校正查找表
GX_DEV_HANDLE g_device = NULL;    ///< 设备句柄
GX_FRAME_DATA g_frame_data = {0}; ///< 采集图像参数
pthread_t g_acquire_thread = 0;   ///< 采集线程ID
bool g_get_image = false;         ///< 采集线程是否结束的标志：true 运行；false 退出
ros::Publisher pub_img_left;
ros::Publisher pub_img_right;
ros::Publisher pub_img_left_gray;
ros::Publisher pub_img_right_gray;
sensor_msgs::Image img_left, img_right;
sensor_msgs::Image img_left_gray, img_right_gray;
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
void rbg24togray(uchar *imgrgb, uchar *imggray, size_t nWidth, size_t nHeight);
void GammaCorrectiom(uchar *src, int iWidth, int iHeight, uchar *Dst);
void BuildTable(float fPrecompensation);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cam");
    ros::NodeHandle n;

    pub_img_left = n.advertise<sensor_msgs::Image>("/image/left", 1000);
    pub_img_right = n.advertise<sensor_msgs::Image>("/image/right", 1000);
    pub_img_left_gray = n.advertise<sensor_msgs::Image>("/image/left_gray", 1000);
    pub_img_right_gray = n.advertise<sensor_msgs::Image>("/image/right_gray", 1000);
    BuildTable(1.0 / 2.2);
    uid_t user = 0;
    user = geteuid();
    // if (user != 0)
    // {
    //     printf("\n");
    //     printf("Please run this application with 'sudo -E ./GxAcquireContinuous' or"
    //            " Start with root !\n");
    //     printf("\n");
    //     return 0;
    // }

    printf("\n");
    printf("-------------------------------------------------------------\n");
    printf("sample to show how to acquire image continuously.\n");
#ifdef __x86_64__
    printf("version: 1.0.1605.8041\n");
#elif __i386__
    printf("version: 1.0.1605.9041\n");
#endif
    printf("-------------------------------------------------------------\n");
    printf("\n");

    printf("Press [x] or [X] and then press [Enter] to Exit the Program\n");
    printf("Initializing......");
    printf("\n\n");

    usleep(2000000);

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
    status = GXSetFloat(g_device, GX_FLOAT_GAIN, 0);

    // 设置白平衡系数
    status = GXSetEnum(g_device, GX_ENUM_BALANCE_RATIO_SELECTOR,
                       GX_BALANCE_RATIO_SELECTOR_BLUE);
    double B = 1.424984;
    double R = 0.988648;
    //     double B = 1;
    // double R = 1;
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
    // double shutterTime = 11111.1;
    double shutterTime = 30000.0;
    status = GXSetFloat(g_device, GX_FLOAT_EXPOSURE_TIME,
                        shutterTime);
    // 设置采集帧率
    status = GXSetEnum(g_device, GX_ENUM_ACQUISITION_FRAME_RATE_MODE,
                       GX_ACQUISITION_FRAME_RATE_MODE_ON);
    status = GXSetFloat(g_device, GX_FLOAT_ACQUISITION_FRAME_RATE, 5.0);
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
    m_rgb_image = new uchar[2560 * 1024 * 3];
    ret = pthread_create(&g_acquire_thread, 0, ProcGetImage, 0);
    // 左图
    img_left.header.frame_id = "cam0";
    img_left.height = 512;
    img_left.width = 640;
    img_left.encoding = "bgr8";
    img_left.is_bigendian = false;
    img_left.step = 640 * 3;
    img_left.data.resize(img_left.step * 512);
    // 右图
    img_right.header.frame_id = "cam1";
    img_right.height = 512;
    img_right.width = 640;
    img_right.encoding = "bgr8";
    img_right.is_bigendian = false;
    img_right.step = 640 * 3;
    img_right.data.resize(img_right.step * 512);
    // 左图灰度
    img_left_gray.header.frame_id = "cam0";
    img_left_gray.height = 512;
    img_left_gray.width = 640;
    img_left_gray.encoding = "mono8";
    img_left_gray.is_bigendian = false;
    img_left_gray.step = 640;
    img_left_gray.data.resize(img_left_gray.step * 512);
    // 右图灰度
    img_right_gray.header.frame_id = "cam1";
    img_right_gray.height = 512;
    img_right_gray.width = 640;
    img_right_gray.encoding = "mono8";
    img_right_gray.is_bigendian = false;
    img_right_gray.step = 640;
    img_right_gray.data.resize(img_right_gray.step * 512);
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
    while (g_get_image)
    {
        if (g_frame_data.pImgBuf == NULL)
        {
            continue;
        }

        status = GXGetImage(g_device, &g_frame_data, 100);
        if (status == GX_STATUS_SUCCESS)
        {
            if (g_frame_data.nStatus == 0)
            {
                auto tmp_time = ros::Time::now();
                // 左图
                img_left.header.stamp = tmp_time;
                // 右图
                img_right.header.stamp = tmp_time;
                //左图灰度图
                img_right_gray.header.stamp = tmp_time;
                //右图灰度图
                img_left_gray.header.stamp = tmp_time;
                // 修改这个的高和宽是否可以直接改变图像大小？
                DxRaw8toRGB24(g_frame_data.pImgBuf, m_rgb_image, g_frame_data.nWidth, g_frame_data.nHeight, RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(BAYERBG), false);
                // memcpy(reinterpret_cast<uchar *>(&img.data[0]), m_rgb_image, g_frame_data.nWidth * g_frame_data.nHeight * 3);
                resize_spilt(reinterpret_cast<uchar *>(&img_right.data[0]), reinterpret_cast<uchar *>(&img_left.data[0]), m_rgb_image, g_frame_data.nWidth, g_frame_data.nHeight);
                rbg24togray(reinterpret_cast<uchar *>(&img_right.data[0]), reinterpret_cast<uchar *>(&img_right_gray.data[0]), 640, 512);
                GammaCorrectiom(reinterpret_cast<uchar *>(&img_right_gray.data[0]), 640, 512, reinterpret_cast<uchar *>(&img_right_gray.data[0]));
                rbg24togray(reinterpret_cast<uchar *>(&img_left.data[0]), reinterpret_cast<uchar *>(&img_left_gray.data[0]), 640, 512);
                GammaCorrectiom(reinterpret_cast<uchar *>(&img_left_gray.data[0]), 640, 512, reinterpret_cast<uchar *>(&img_left_gray.data[0]));
                // 图像分割直接半行半行memcpy
                pub_img_right.publish(img_right);
                pub_img_left.publish(img_left);
                pub_img_right_gray.publish(img_right_gray);
                pub_img_left_gray.publish(img_left_gray);
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