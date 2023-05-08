#include "../include/Gxdrive2.h"
typedef void *(*THREADFUNCPTR)(void *);
int main(int argc, char **argv)
{
    ros::init(argc, argv, "cam");
    Gxdrive cam;
    cam.initLog("/share/cam_ws/src/cam/logs/");
    GX_STATUS status;
    cam.yaml_read("/share/cam_ws/src/cam/camera.yaml");
    printf("Initializing......");
    printf("\n\n");
    usleep(1000000);
    cam.Init();
    cam.remap_Init();
    cam.OpenGx();
    int ret = pthread_create(&cam.g_acquire_thread, 0, (THREADFUNCPTR)&Gxdrive::ProcGetImage, &cam);
    if (ret != 0)
    {
        printf("<Failed to create the collection thread>\n");
        status = GXCloseDevice(cam.g_device);
        if (cam.g_device != NULL)
        {
            cam.g_device = NULL;
        }
        status = GXCloseLib();
        return 0;
    }
    // ret = pthread_create(&cam.depth_thread, 0, (THREADFUNCPTR)&Gxdrive::depth_map, &cam);
    // if (ret != 0)
    // {
    //     printf("<Failed to create the collection thread>\n");
    //     status = GXCloseDevice(cam.g_device);
    //     if (cam.g_device != NULL)
    //     {
    //         cam.g_device = NULL;
    //     }
    //     status = GXCloseLib();
    //     return 0;
    // }
    ros::spin();
    // 为停止采集做准备
    cam.UnPreForImage();
    // 关闭设备
    status = GXCloseDevice(cam.g_device);

    // 释放库
    status = GXCloseLib();
    return 0;
}