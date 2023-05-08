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

#include <vpi/OpenCVInterop.hpp>
#include <vpi/Image.h>
#include <vpi/Status.h>
#include <vpi/Stream.h>
#include <vpi/algo/ConvertImageFormat.h>
#include <vpi/algo/Rescale.h>
// extern "C"
//{
#include <NvBufSurface.h>
#include <nvbuf_utils.h>
#include <NvBuffer.h>
#include <cuda_runtime_api.h>
#include <cuda.h>
#include "nvbufsurface.h"
#include "nvbufsurftransform.h"
//}
using namespace std;
using namespace cv;
using namespace cv::dnn;

#define MEMORY_ALLOT_ERROR -1
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

GX_DEV_HANDLE g_device = NULL;    ///< 设备句柄
GX_FRAME_DATA g_frame_data = {0}; ///< 采集图像参数
pthread_t g_acquire_thread = 0;   ///< 采集线程ID
bool g_get_image = false;         ///< 采集线程是否结束的标志：true 运行；false 退出
ros::Publisher pub_img_left;
ros::Publisher pub_img_right;
sensor_msgs::Image img_left, img_right;
uchar *m_rgb_image = NULL; // 增加的内容

// NVIDIA vpi
VPIImageWrapperParams nvpar;
// VPIImage vpiimage = NULL;
VPIImage vpiimage_scale = NULL;
VPIImage vpiimage_gray = NULL;
VPIImage vpiimage_bgr = NULL;
VPIImage vpiimage_right = NULL;
VPIImage vpiimage_left = NULL;
VPIStream stream = NULL;
// int *dma_out;
int dma_in = -1;
NvBufferTransformParams Nv_param;
VPIRectangleI clipBounds_left, clipBounds_right;
NvBufSurf::NvCommonAllocateParams camparams;
NvBufSurface *pSurf = NULL;

void convert_raw8_to_nvbuffer(unsigned char *raw8_data, int width, int height, NvBuffer *nvbuf)
{
    // Set NvBuffer parameters
    NvBufferCreateParams params = {0};
    params.width = 2560;
    params.height = 1024;
    params.layout = NvBufferLayout_Pitch;
    params.colorFormat = NvBufferColorFormat_NV12;
    params.payloadType = NvBufferPayload_SurfArray;
    params.nvbuf_tag = NvBufferTag_NONE;

    // Create NvBuffer
    NvBufferCreateEx(0, &params);

    int pitch;
    NvBufferGetParams(0, &params);
    pitch = params.pitch[0];

    // Map the buffer for CPU access
    void *mapped_data;
    NvBufferMemMap(0, 0, NvBufferMem_Write, &mapped_data);

    // Copy the RAW8 image to the buffer
    for (int row = 0; row < 1024; ++row)
    {
        const void *src_row = &raw8_data[row * 2560];
        void *dst_row = static_cast<char *>(mapped_data) + row * pitch;
        memcpy(dst_row, src_row, 2560);
    }

    // Unmap the buffer
    NvBufferMemUnMap(0, 0, &mapped_data);
}

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

    pub_img_left = n.advertise<sensor_msgs::Image>("/image/left", 1000);
    pub_img_right = n.advertise<sensor_msgs::Image>("/image/right", 1000);
    // NVDIA vpi
    // vpiInitImageWrapperParams(&nvpar);
    // Nv_param.transform_flag = 0;
    // VPIBackend backend = VPI_BACKEND_CPU;
    // // CHECK_STATUS(vpiImageCreate(2560, 1024, VPI_IMAGE_FORMAT_BGRA8, 0, &vpiimage));
    // // CHECK_STATUS(vpiImageCreate(2560, 1024, VPI_IMAGE_FORMAT_BGR8, 0, &vpiimage_bgr));
    // CHECK_STATUS(vpiImageCreate(1920, 768, VPI_IMAGE_FORMAT_BGR8, 0, &vpiimage_scale));
    // CHECK_STATUS(vpiImageCreate(1920, 768, VPI_IMAGE_FORMAT_Y8, 0, &vpiimage_gray));
    // CHECK_STATUS(vpiImageCreate(960, 768, VPI_IMAGE_FORMAT_BGR8, 0, &vpiimage_right));
    // CHECK_STATUS(vpiImageCreate(960, 768, VPI_IMAGE_FORMAT_BGR8, 0, &vpiimage_left));
    // CHECK_STATUS(vpiStreamCreate(backend | VPI_BACKEND_CUDA, &stream));

    // camparams.memType = NVBUF_MEM_SURFACE_ARRAY;
    // camparams.height = 1024;
    // camparams.width = 2560;
    // camparams.memtag = NvBufSurfaceTag_CAMERA;
    // camparams.layout = NVBUF_LAYOUT_PITCH;
    // camparams.colorFormat = NvBufSurfaceColorFormat::NVBUF_COLOR_FORMAT_ARGB;

    // params.memType = NVBUF_MEM_SURFACE_ARRAY;
    // if (ctx->out_pixfmt == 1)
    //   params.colorFormat = NVBUF_COLOR_FORMAT_NV12;
    // else if (ctx->out_pixfmt == 2)
    //   params.colorFormat = NVBUF_COLOR_FORMAT_YUV420;
    // else if (ctx->out_pixfmt == 3)
    //   params.colorFormat = NVBUF_COLOR_FORMAT_NV16;
    // else if (ctx->out_pixfmt == 4)
    //   params.colorFormat = NVBUF_COLOR_FORMAT_NV24;

    // clipBounds_left.x = 0;
    // clipBounds_left.y = 0;
    // clipBounds_left.width = 960;
    // clipBounds_left.height = 768;

    // clipBounds_right.x = 960;
    // clipBounds_right.y = 0;
    // clipBounds_right.width = 960;
    // clipBounds_right.height = 768;

    uid_t user = 0;
    user = geteuid();
    if (user != 0)
    {
        printf("\n");
        printf("Please run this application with 'sudo -E ./GxAcquireContinuous' or"
               " Start with root !\n");
        printf("\n");
        return 0;
    }

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

    usleep(1000000);

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
    img_left.header.frame_id = "cam";
    img_left.height = 768;
    img_left.width = 960;
    img_left.encoding = "bgr8";
    img_left.is_bigendian = false;
    img_left.step = 960 * 3;
    img_left.data.resize(img_left.step * 768);
    // 右图
    img_right.header.frame_id = "cam";
    img_right.height = 768;
    img_right.width = 960;
    img_right.encoding = "bgr8";
    img_right.is_bigendian = false;
    img_right.step = 960 * 3;
    img_right.data.resize(img_right.step * 768);
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
                img_left.header.stamp = ros::Time::now();
                // 右图
                img_right.header.stamp = ros::Time::now();
                // Raw2NvBuffer
                // if (NvBufSurf::NvAllocate(&camparams, 1, &dma_in))
                //     std::cout << "error" << std::endl;

                // ROS_INFO("1");
                // NvBufSurfaceFromFd(dma_in, (void **)(&pSurf));
                // NvBufSurfaceSyncForDevice(pSurf, 0, 0);
                // std::cout << pSurf->memType << std::endl;
                // ROS_INFO("2");
                // // NvBufferCreate(dma_in, g_frame_data.nWidth, g_frame_data.nHeight, NvBufferLayout_Pitch, NvBufferColorFormat_ABGR32);
                // Raw2NvBufSurface((unsigned char *)g_frame_data.pImgBuf, 0, 0, g_frame_data.nWidth, g_frame_data.nHeight, pSurf);
                // ROS_INFO("3");

                // gray
                // NvBufferCreate(dma_out, g_frame_data.nWidth, g_frame_data.nHeight, NvBufferLayout_Pitch, NvBufferColorFormat_ABGR32);
                // NvBufferTransform(*dma_in, *dma_out, &Nv_param);
                // create VPIImageData
                // DxRaw8toRGB24(g_frame_data.pImgBuf, m_rgb_image, g_frame_data.nWidth, g_frame_data.nHeight, RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(BAYERBG), false);
                NvBuffer *buffer = nullptr;
                convert_raw8_to_nvbuffer(g_frame_data.pImgBuf, 2560, 1024, mybuf);
                // VPIImageData data = {};
                // memset(&data, 0, sizeof(data));
                // data.bufferType = VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR;
                // data.buffer.pitch.format = VPI_IMAGE_FORMAT_BGR8;
                // data.buffer.pitch.numPlanes = 1;
                // ROS_INFO("VPI_IMAGE_FORMAT_BGR8:%d", vpiImageFormatGetPlaneCount(VPI_IMAGE_FORMAT_BGR8));
                // VPIPixelType pixType = vpiImageFormatGetPlanePixelType(VPI_IMAGE_FORMAT_BGR8, 0);
                // VPIImagePlanePitchLinear &plane = data.buffer.pitch.planes[0];
                // plane.width = 2560;
                // plane.height = 1024;
                // plane.pitchBytes = 3 * 2560;
                // plane.pixelType = pixType;
                // plane.data = m_rgb_image;
                // // std::cout << (int)*(uchar *)plane.data << std::endl;
                // // std::cout << (int)*((uchar *)plane.data + 1) << std::endl;
                // // std::cout << (int)*((uchar *)plane.data + 2) << std::endl;
                // // std::cout << (int)*((uchar *)plane.data + 3) << std::endl;
                // // std::cout << (int)*((uchar *)plane.data + 4) << std::endl;
                // // ROS_INFO("%ld", data.buffer.pitch.format);
                // // ROS_INFO("%ld", VPI_IMAGE_FORMAT_BGR8);

                //CHECK_STATUS(vpiImageCreate(&data, &nvpar, VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &vpiimage));
                // // std::cout << "bbbb" << std::endl;
                // // Finally, convert the result back to BGR8
                // // CHECK_STATUS(vpiSubmitConvertImageFormat(stream, VPI_BACKEND_CUDA, vpiimage, vpiimage_bgr, NULL));
                // CHECK_STATUS(vpiImageCreateWrapper(&data, nullptr, 0, &vpiimage_bgr));
                // // ROS_INFO("%d", nv::vpi::detail::ToOpenCVType(pixType));
                // // VPIImageFormat format;
                // // ROS_INFO("%d", vpiImageGetFormat(vpiimage_bgr, &format));
                // // if (format == VPI_IMAGE_FORMAT_BGR8)
                // //     ROS_INFO("right");
                // // ROS_INFO("%d", vpiImageGetFormat(vpiimage_scale, &format));
                // // if (format == VPI_IMAGE_FORMAT_BGR8)
                // //     ROS_INFO("right");
                // // downsample
                // int imgFlags = VPI_BACKEND_CPU | VPI_BACKEND_CUDA;
                // CHECK_STATUS(vpiSubmitRescale(stream, VPI_BACKEND_CPU, vpiimage_bgr, vpiimage_scale, VPI_INTERP_LINEAR, VPI_BORDER_CLAMP, 0));
                // // bgr8 to gray
                // // CHECK_STATUS(vpiSubmitConvertImageFormat(stream, VPI_BACKEND_CUDA, vpiimage_spilt, vpiimage_gray, NULL));
                // // Wait until the algorithm finishes processing
                // CHECK_STATUS(vpiStreamSync(stream));
                // // spilt
                // CHECK_STATUS(vpiImageCreateView(vpiimage_scale, &clipBounds_right, VPI_BACKEND_CUDA, &vpiimage_right));
                // ROS_INFO("3");
                // CHECK_STATUS(vpiImageCreateView(vpiimage_scale, &clipBounds_left, VPI_BACKEND_CUDA, &vpiimage_left));
                // ROS_INFO("4");
                // // pubilish
                // {
                //     // Lock output image to retrieve its data on cpu memory
                //     VPIImageData outData_right;
                //     CHECK_STATUS(vpiImageLockData(vpiimage_right, VPI_LOCK_READ, VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &outData_right));
                //     // Returned data consists of host-accessible memory buffers in pitch-linear layout.
                //     assert(outData_right.bufferType == VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR);
                //     VPIImageBufferPitchLinear &outDataPitch_right = outData_right.buffer.pitch;
                //     memcpy(reinterpret_cast<uchar *>(&img_right.data[0]), outDataPitch_right.planes[0].data, 3 * 960);
                //     // Done handling output image, don't forget to unlock it.
                //     CHECK_STATUS(vpiImageUnlock(vpiimage_right));
                //     VPIImageData outData_left;
                //     CHECK_STATUS(vpiImageLockData(vpiimage_left, VPI_LOCK_READ, VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &outData_left));
                //     // Returned data consists of host-accessible memory buffers in pitch-linear layout.
                //     assert(outData_left.bufferType == VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR);
                //     VPIImageBufferPitchLinear &outDataPitch_left = outData_left.buffer.pitch;
                //     memcpy(reinterpret_cast<uchar *>(&img_left.data[0]), outDataPitch_left.planes[0].data, 3 * 960);
                //     // Done handling output image, don't forget to unlock it.
                //     CHECK_STATUS(vpiImageUnlock(vpiimage_left));
                // }
                pub_img_right.publish(img_right);
                pub_img_left.publish(img_left);
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