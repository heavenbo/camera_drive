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
#include <chrono>
#include "../include/nvidia.hpp"
#include <nppi.h>

using namespace std;
using namespace std::chrono;
using namespace cv;
using namespace cv::dnn;
int source_dmabuf_fd = -1;
int dest_dmabuf_fd_right = -1;
int dest_dmabuf_fd_left = -1;
NvBufSurface *src_nvbuf_surf = 0;
NvBufSurface *tmpbuf = 0;
NvBufSurface *dst_nvbuf_surf_left = 0;
NvBufSurface *dst_nvbuf_surf_right = 0;
NvBufSurface *src_tran_nvbuf_surf = 0;
NvBufSurface *dst_tran_nvbuf_surf = 0;
std::ofstream image;
int image_a = 0;

// debug

#define MEMORY_ALLOT_ERROR -1
#define CHECK_ERROR(condition, error_str)              \
    if (condition)                                     \
    {                                                  \
        if (source_dmabuf_fd != -1)                    \
        {                                              \
            NvBufSurfaceDestroy(src_nvbuf_surf);       \
            source_dmabuf_fd = -1;                     \
        }                                              \
        if (dest_dmabuf_fd_left != -1)                 \
        {                                              \
            NvBufSurfaceDestroy(dst_nvbuf_surf_left);  \
            NvBufSurfaceDestroy(dst_nvbuf_surf_right); \
            dest_dmabuf_fd_left = -1;                  \
            dest_dmabuf_fd_right = -1;                 \
        }                                              \
        cudaStreamDestroy(config_params.cuda_stream);  \
        throw std::runtime_error(error_str);           \
    }

GX_DEV_HANDLE g_device = NULL;    ///< 设备句柄
GX_FRAME_DATA g_frame_data = {0}; ///< 采集图像参数
pthread_t g_acquire_thread = 0;   ///< 采集线程ID
bool g_get_image = false;         ///< 采集线程是否结束的标志：true 运行；false 退出
ros::Publisher pub_img_left;
ros::Publisher pub_img_right;
sensor_msgs::Image img_left, img_right;
uchar *m_rgb_image = NULL; // 增加的内容

// NVIDIA vpi
NvBufSurfaceAllocateParams input_params = {{0}};
NvBufSurfaceAllocateParams output_params_left = {{0}};
NvBufSurfaceAllocateParams output_params_right = {{0}};
NvBufSurfTransformParams transform_params_left = {0};
NvBufSurfTransformParams transform_params_right = {0};
NvBufSurfTransformRect src_rect_left = {0}, dest_rect_left = {0};
NvBufSurfTransformRect src_rect_right = {0}, dest_rect_right = {0};
NvBufSurfTransformConfigParams config_params = {NvBufSurfTransformCompute_VIC, 0, NULL};

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
    image.open("/home/image.txt");

    pub_img_left = n.advertise<sensor_msgs::Image>("/image/left", 1);
    pub_img_right = n.advertise<sensor_msgs::Image>("/image/right", 1);

    int width = 2560;
    int height = 1024;

    input_params.params.width = width;
    input_params.params.height = height;
    input_params.params.memType = NVBUF_MEM_SURFACE_ARRAY;
    input_params.params.gpuId = 0;
    input_params.params.colorFormat = NVBUF_COLOR_FORMAT_GRAY8;
    input_params.memtag = NvBufSurfaceTag_NONE;

    // input_tran_params.params.width = width;
    // input_tran_params.params.height = height;
    // input_tran_params.params.memType = NVBUF_MEM_SURFACE_ARRAY;
    // input_tran_params.params.gpuId = 0;
    // input_tran_params.params.colorFormat = NVBUF_COLOR_FORMAT_BGRA;
    // input_tran_params.memtag = NvBufSurfaceTag_VIDEO_CONVERT;

    // output_tran_params.params.width = width;
    // output_tran_params.params.height = height;
    // output_tran_params.params.memType = NVBUF_MEM_SURFACE_ARRAY;
    // output_tran_params.params.gpuId = 0;
    // output_tran_params.params.colorFormat = NVBUF_COLOR_FORMAT_GRAY8;
    // output_tran_params.memtag = NvBufSurfaceTag_VIDEO_CONVERT;

    output_params_left.params.width = width / 2;
    output_params_left.params.height = height;
    output_params_left.params.memType = NVBUF_MEM_SURFACE_ARRAY;
    output_params_left.params.gpuId = 0;
    output_params_left.params.colorFormat = NVBUF_COLOR_FORMAT_GRAY8;
    output_params_left.memtag = NvBufSurfaceTag_NONE;

    output_params_right.params.width = width / 2;
    output_params_right.params.height = height;
    output_params_right.params.memType = NVBUF_MEM_SURFACE_ARRAY;
    output_params_right.params.gpuId = 0;
    output_params_right.params.colorFormat = NVBUF_COLOR_FORMAT_GRAY8;
    output_params_right.memtag = NvBufSurfaceTag_NONE;

    // fill_bytes_per_pixel(input_params.params.colorFormat, bytes_per_pixel_srcfmt);
    // fill_bytes_per_pixel(output_params.params.colorFormat, bytes_per_pixel_destfmt);

    /* Create the HW Buffer. It is exported as
    ** an FD by the hardware.
    */
    int ret = NvBufSurfaceAllocate(&src_nvbuf_surf, 1, &input_params);
    CHECK_ERROR(ret, "Error in creating the source buffer.");
    src_nvbuf_surf->numFilled = 1;
    source_dmabuf_fd = src_nvbuf_surf->surfaceList[0].bufferDesc;

    // ret = NvBufSurfaceAllocate(&src_tran_nvbuf_surf, 1, &input_tran_params);
    // CHECK_ERROR(ret, "Error in creating the source buffer.");
    // src_tran_nvbuf_surf->numFilled = 1;

    // ret = NvBufSurfaceAllocate(&dst_tran_nvbuf_surf, 1, &output_tran_params);
    // CHECK_ERROR(ret, "Error in creating the destination buffer.");
    // dst_tran_nvbuf_surf->numFilled = 1;

    ret = NvBufSurfaceAllocate(&dst_nvbuf_surf_left, 1, &output_params_left);
    CHECK_ERROR(ret, "Error in creating the destination buffer.");
    dst_nvbuf_surf_left->numFilled = 1;
    dest_dmabuf_fd_left = dst_nvbuf_surf_left->surfaceList[0].bufferDesc;

    ret = NvBufSurfaceAllocate(&dst_nvbuf_surf_right, 1, &output_params_right);
    CHECK_ERROR(ret, "Error in creating the destination buffer.");
    dst_nvbuf_surf_right->numFilled = 1;
    dest_dmabuf_fd_right = dst_nvbuf_surf_right->surfaceList[0].bufferDesc;

    config_params.gpu_id = 0;
    config_params.compute_mode = NvBufSurfTransformCompute_GPU;
    cudaStreamCreateWithFlags(&(config_params.cuda_stream), cudaStreamNonBlocking);

    /* Set the session parameters */
    ret = NvBufSurfTransformSetSessionParams(&config_params);
    CHECK_ERROR(ret, "Error in NvBufSurfTransformSetSessionParams");

    /* Transformation parameters are now defined
    ** which is passed to the NvBuuferTransform
    ** for required conversion.
    */

    src_rect_left.top = 0;
    src_rect_left.left = 0;
    src_rect_left.width = width / 2;
    src_rect_left.height = height;
    dest_rect_left.top = 0;
    dest_rect_left.left = 0;
    dest_rect_left.width = width / 2;
    dest_rect_left.height = height;

    src_rect_right.top = 0;
    src_rect_right.left = width / 2;
    src_rect_right.width = width / 2;
    src_rect_right.height = height;
    dest_rect_right.top = 0;
    dest_rect_right.left = 0;
    dest_rect_right.width = width / 2;
    dest_rect_right.height = height;

    /* @transform_flag defines the flags for
    ** enabling the valid transforms.
    ** All the valid parameters are present in
    ** the nvbufsurface header.
    */

    memset(&transform_params_left, 0, sizeof(transform_params_left));
    transform_params_left.transform_flag = NVBUFSURF_TRANSFORM_CROP_SRC;
    transform_params_left.transform_flip = NvBufSurfTransform_None;
    transform_params_left.transform_filter = NvBufSurfTransformInter_Nearest;
    transform_params_left.src_rect = &src_rect_left;
    transform_params_left.dst_rect = &dest_rect_left;

    memset(&transform_params_right, 0, sizeof(transform_params_right));
    transform_params_right.transform_flag = NVBUFSURF_TRANSFORM_CROP_SRC; // NVBUFSURF_TRANSFORM_FILTER | NVBUFSURF_TRANSFORM_FLIP;
    transform_params_right.transform_flip = NvBufSurfTransform_None;
    transform_params_right.transform_filter = NvBufSurfTransformInter_Nearest;
    transform_params_right.src_rect = &src_rect_right;
    transform_params_right.dst_rect = &dest_rect_right;

    // usleep(1000000);

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
    m_rgb_image = new uchar[2560 * 1024];
    ret = pthread_create(&g_acquire_thread, 0, ProcGetImage, 0);
    // 左图
    img_left.header.frame_id = "cam";
    img_left.height = 1024;
    img_left.width = 1280;
    img_left.encoding = "mono8";
    img_left.is_bigendian = false;
    img_left.step = 1280;
    img_left.data.resize(img_left.step * 1024);
    // 右图
    img_right.header.frame_id = "cam";
    img_right.height = 1024;
    img_right.width = 1280;
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
        status = GXGetImage(g_device, &g_frame_data, 1000);
        if (status == GX_STATUS_SUCCESS)
        {
            if (g_frame_data.nStatus == 0)
            {
                auto start = system_clock::now();
                // 左图
                img_left.header.stamp = ros::Time::now();
                // 右图
                img_right.header.stamp = ros::Time::now();

                int ret = 0;

                /* Void data pointer to store memory-mapped
                ** virtual addresses of the planes.
                */
                void *virtualip_data_addr;
                unsigned int plane = 0;
                Raw2NvBufSurface((u_char *)g_frame_data.pImgBuf, 0, 0, 2560, 1024, src_nvbuf_surf);

                ret = NvBufSurfTransform(src_nvbuf_surf, dst_nvbuf_surf_left, &transform_params_left);
                CHECK_ERROR(ret, "Error in transformation.");

                ret = NvBufSurfTransform(src_nvbuf_surf, dst_nvbuf_surf_right, &transform_params_right);
                CHECK_ERROR(ret, "Error in transformation.");
                // copy
                // ret = NvBufSurfaceCopy(dst_tran_nvbuf_surf, dst_nvbuf_surf);
                // CHECK_ERROR(ret, "Error in NvBufSurfaceCopy");

                // 转出去
                NvBufSurface *nvbuf_surf_out_left = 0;

                ret = NvBufSurfaceFromFd(dest_dmabuf_fd_left, (void **)(&nvbuf_surf_out_left));
                CHECK_ERROR(ret, "NvBufSurfaceFromFd failed");

                NvBufSurface *nvbuf_surf_out_right = 0;
                ret = NvBufSurfaceFromFd(dest_dmabuf_fd_right, (void **)(&nvbuf_surf_out_right));
                CHECK_ERROR(ret, "NvBufSurfaceFromFd failed");

                /* Void data pointer to store memory-mapped
                ** virtual addresses of the planes.
                */

                // void *virtualop_data_addr;
                // for (plane = 0; plane < nvbuf_surf_out->surfaceList[0].planeParams.num_planes; ++plane)
                // {
                //     ret = NvBufSurfaceMap(nvbuf_surf_out, 0, plane, NVBUF_MAP_READ);
                //     if (ret == 0)
                //     {
                //         // NvBufSurfaceSyncForCpu(nvbuf_surf_out, 0, plane);
                //         virtualop_data_addr = (void *)nvbuf_surf_out->surfaceList[0].mappedAddr.addr[plane];

                //         memcpy(reinterpret_cast<uchar *>(&img_left.data[0]), (char *)virtualop_data_addr, 1024 * 2560);
                //     }
                //     else
                //     {
                //         std::cout << "out error" << std::endl;
                //     }
                //     NvBufSurfaceUnMap(nvbuf_surf_out, 0, plane);
                // }
                NvBufSurface2Raw(nvbuf_surf_out_left, 0, 0, 1280, 1024, reinterpret_cast<uchar *>(&img_left.data[0]));
                NvBufSurface2Raw(nvbuf_surf_out_right, 0, 0, 1280, 1024, reinterpret_cast<uchar *>(&img_right.data[0]));
                pub_img_left.publish(img_left);
                pub_img_right.publish(img_right);
                // std::cout << "image_a" << image_a << std::endl;
                auto end = system_clock::now();
                auto duration = duration_cast<microseconds>(end - start);
                cout << "花费了"
                     << double(duration.count()) * microseconds::period::num / microseconds::period::den << "秒" << endl;

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