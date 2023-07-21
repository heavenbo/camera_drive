#include "Gxdrive2.h"

void Gxdrive::yaml_read(std::string file_path)
{
    YAML::Node yaml_node = YAML::LoadFile(file_path);
    this->sync_frame = yaml_node["Sync_frame"].as<int>();
    this->camera_style = yaml_node["Camera_style"].as<std::string>();
    this->control_mode = yaml_node["Control_mode"].as<int>();
    this->frequency = yaml_node["Frequency"].as<int>();
    this->gain = yaml_node["Gain"].as<double>();
    this->pwm_period = 1000 / yaml_node["Pwm_fre"].as<double>();
    this->pwm_period_s = pwm_period / 1000;
    this->pwm_duty = yaml_node["Duty"].as<double>();
    this->pwm_on = this->pwm_duty * this->pwm_period;
    this->exposure_t = yaml_node["Exposure_T"].as<double>();
    this->bal_r = yaml_node["Balance_ratio"]["r"].as<double>();
    this->bal_g = yaml_node["Balance_ratio"]["g"].as<double>();
    this->bal_b = yaml_node["Balance_ratio"]["b"].as<double>();
    for (int i = 0; i < 4; i++)
    {
        this->left_intrinsics[i] = yaml_node["Left_cam"]["intrinsics"][i].as<double>();
        this->right_intrinsics[i] = yaml_node["Right_cam"]["intrinsics"][i].as<double>();
        this->left_distortion[i] = yaml_node["Left_cam"]["distortion"][i].as<double>();
        this->right_distortion[i] = yaml_node["Right_cam"]["distortion"][i].as<double>();
    }
    for (int i = 0; i < 12; i++)
    {
        this->t_cn_cnm1[i] = yaml_node["T_cn_cnm1"][i / 4][i % 4].as<double>();
    }
    this->height = yaml_node["Height"].as<int>();
    this->width = yaml_node["Width"].as<int>();
}

void Gxdrive::Init()
{
    // 初始化指针
    cudaMallocManaged(&this->tmp_color, 2560 * 1024 * 24);
    cudaMallocManaged(&this->left_rgb, this->width * this->height * 24);
    cudaMallocManaged(&this->right_rgb, this->width * this->height * 24);
    cudaMallocManaged(&this->right_gamma, this->width * this->height * 24);
    cudaMallocManaged(&this->left_gamma, this->width * this->height * 24);
    cudaMallocManaged(&this->right_gray, this->width * this->height * 8);
    cudaMallocManaged(&this->left_gray, this->width * this->height * 8);
    // ros话题发布者初始化
    this->pub_img_left = this->n.advertise<sensor_msgs::Image>("/cam/left/bgr", 1000);
    this->pub_img_right = this->n.advertise<sensor_msgs::Image>("/cam/right/bgr", 1000);
    this->pub_img_left_gray = this->n.advertise<sensor_msgs::Image>("/cam/left/gray", 1000);
    this->pub_img_right_gray = this->n.advertise<sensor_msgs::Image>("/cam/right/gray", 1000);
    this->pub_stereo_disparity = this->n.advertise<sensor_msgs::Image>("/depth/stereo_disparity", 1000);
    // ros服务
    this->tricon_client = n.serviceClient<mavros_msgs::CommandTriggerControl>("/mavros/cmd/trigger_control");
    this->trival_client = n.serviceClient<mavros_msgs::CommandTriggerInterval>("/mavros/cmd/trigger_interval");
    // ros话题订阅
    this->sub_time = this->n.subscribe<mavros_msgs::CamIMUStamp>("/mavros/sync/cam_imu_stamp", 100, boost::bind(&Gxdrive::Time_callback, this, _1));
    // 控制pwm输出 control_mode为1时
    if (this->control_mode == 1)
    {
        this->trigger_interval.request.cycle_time = this->pwm_period;
        this->trigger_interval.request.integration_time = this->pwm_on;
        this->trival_client.call(this->trigger_interval);
        this->trigger_control.request.sequence_reset = true;
        this->trigger_control.request.trigger_enable = false;
        this->trigger_control.request.trigger_pause = false;
        this->tricon_client.call(this->trigger_control);
        if (control_mode == 1)
        {
            this->trigger_control.request.sequence_reset = false;
            this->trigger_control.request.trigger_enable = true;
            this->trigger_control.request.trigger_pause = false;
        }
        // 初始化sensorimage
        //  左图
        this->img_left.header.frame_id = "cam0";
        this->img_left.height = this->height;
        this->img_left.width = this->width;
        this->img_left.encoding = "rgb8";
        this->img_left.is_bigendian = false;
        this->img_left.step = this->width * 3;
        this->img_left.data.resize(this->img_left.step * this->height);
        // 右图
        this->img_right.header.frame_id = "cam1";
        this->img_right.height = this->height;
        this->img_right.width = this->width;
        this->img_right.encoding = "rgb8";
        this->img_right.is_bigendian = false;
        this->img_right.step = this->width * 3;
        this->img_right.data.resize(this->img_right.step * this->height);
        // 左图灰度
        this->img_left_gray.header.frame_id = "cam0";
        this->img_left_gray.height = this->height;
        this->img_left_gray.width = this->width;
        this->img_left_gray.encoding = "mono8";
        this->img_left_gray.is_bigendian = false;
        this->img_left_gray.step = this->width;
        this->img_left_gray.data.resize(this->img_left_gray.step * this->height);
        // 右图灰度
        this->img_right_gray.header.frame_id = "cam1";
        this->img_right_gray.height = this->height;
        this->img_right_gray.width = this->width;
        this->img_right_gray.encoding = "mono8";
        this->img_right_gray.is_bigendian = false;
        this->img_right_gray.step = this->width;
        this->img_right_gray.data.resize(this->img_right_gray.step * this->height);
        // 视差图
        this->stereo_disparity.header.frame_id = "cam";
        this->stereo_disparity.height = this->height;
        this->stereo_disparity.width = this->width;
        this->stereo_disparity.encoding = "mono8";
        this->stereo_disparity.is_bigendian = false;
        this->stereo_disparity.step = this->width;
        // this->stereo_disparity.step = this->width ;
        this->stereo_disparity.data.resize(this->stereo_disparity.step * this->height);
    }

    void Gxdrive::remap_Init()
    {
        cv::Mat mapLx, mapLy, mapRx, mapRy;
        cv::Mat Rl, Rr, Pl, Pr, Q;

        cudaMallocManaged(&pLxmap, width * height * 32);
        cudaMallocManaged(&pLymap, width * height * 16);
        cudaMallocManaged(&pRxmap, width * height * 32);
        cudaMallocManaged(&pRymap, width * height * 16);

        cudaMallocManaged(&left_stereo, width * height * 8);
        cudaMallocManaged(&right_stereo, width * height * 8);

        cv::Mat K_0 = (cv::Mat_<double>(3, 3) << left_intrinsics[0], 0.0, left_intrinsics[2], 0.0, left_intrinsics[1], left_intrinsics[3], 0.0, 0.0, 1.0);
        cv::Mat K_1 = (cv::Mat_<double>(3, 3) << right_intrinsics[0], 0.0, right_intrinsics[2], 0.0, right_intrinsics[1], right_intrinsics[3], 0.0, 0.0, 1.0);
        cv::Mat Dist_0 = (cv::Mat_<double>(4, 1) << left_distortion[0], left_distortion[1], left_distortion[2], left_distortion[3]);
        cv::Mat Dist_1 = (cv::Mat_<double>(4, 1) << right_distortion[0], right_distortion[1], right_distortion[2], right_distortion[3]);
        // 设定两个相机之间的旋转和平移, R为Rrl
        cv::Mat R = (cv::Mat_<double>(3, 3) << t_cn_cnm1[0], t_cn_cnm1[1], t_cn_cnm1[2],
                     t_cn_cnm1[4], t_cn_cnm1[5], t_cn_cnm1[6],
                     t_cn_cnm1[8], t_cn_cnm1[9], t_cn_cnm1[10]);
        // 从平移的数值应该能看出来, t为 左目相机在右目相机的坐标系中的位置
        cv::Mat T = (cv::Mat_<double>(3, 1) << t_cn_cnm1[3], t_cn_cnm1[7], t_cn_cnm1[11]);

        cv::stereoRectify(K_0, Dist_0, K_1, Dist_1, cv::Size(width, height), R, T, Rl, Rr, Pl, Pr, Q);

        cv::initUndistortRectifyMap(K_0, Dist_0, Rl, Pl, cv::Size(width, height), CV_32FC1, mapLx, mapLy);
        cv::initUndistortRectifyMap(K_1, Dist_1, Rr, Pr, cv::Size(width, height), CV_32FC1, mapRx, mapRy);
        // std::cout << mapLx.step[0] << std::endl;
        // std::cout << (float)*mapLx.data << std::endl;
        memcpy(pLxmap, (float *)mapLx.data, width * height * 4);
        // std::cout << *pLxmap << std::endl;
        // std::cout << (float *)pLxmap << std::endl;
        memcpy(pLymap, (float *)mapLy.data, width * height * 4);
        memcpy(pRxmap, (float *)mapRx.data, width * height * 4);
        memcpy(pRymap, (float *)mapRy.data, width * height * 4);
    }

    GX_STATUS Gxdrive::OpenGx()
    {
        GX_OPEN_PARAM open_param;
        uint32_t device_num = 0;
        uint32_t ret = 0;
        GX_STATUS status;
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
            status = GXOpenDevice(&open_param, &this->g_device);
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
        // 设置最小增益值
        status = GXSetEnum(this->g_device, GX_ENUM_GAIN_SELECTOR,
                           GX_GAIN_SELECTOR_ALL);
        status = GXSetFloat(this->g_device, GX_FLOAT_GAIN, this->gain);

        // 设置白平衡系数
        status = GXSetEnum(this->g_device, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_BLUE);
        status = GXSetFloat(this->g_device, GX_FLOAT_BALANCE_RATIO, bal_b);

        status = GXSetEnum(this->g_device, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_GREEN);
        status = GXSetFloat(this->g_device, GX_FLOAT_BALANCE_RATIO, bal_g);

        status = GXSetEnum(this->g_device, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED);
        status = GXSetFloat(this->g_device, GX_FLOAT_BALANCE_RATIO, bal_r);
        // 设置采集模式为连续采集
        status = GXSetEnum(this->g_device, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
        // 引脚选择为 Line2
        status = GXSetEnum(this->g_device, GX_ENUM_LINE_SELECTOR, GX_ENUM_LINE_SELECTOR_LINE2);
        // 设置引脚方向为输入
        status = GXSetEnum(this->g_device, GX_ENUM_LINE_MODE, GX_ENUM_LINE_MODE_INPUT);
        // 根据control_mode选择出发模式
        status = GXSetEnum(this->g_device, GX_ENUM_TRIGGER_MODE, control_mode);
        // 设置触发激活方式为上升沿
        status = GXSetEnum(this->g_device, GX_ENUM_TRIGGER_ACTIVATION,
                           GX_TRIGGER_ACTIVATION_RISINGEDGE);
        // 设置上升沿滤波最小值
        GX_FLOAT_RANGE raisingRange;
        status = GXGetFloatRange(this->g_device, GX_FLOAT_TRIGGER_FILTER_RAISING, &raisingRange);
        status = GXSetFloat(this->g_device, GX_FLOAT_TRIGGER_FILTER_RAISING, raisingRange.dMax);
        // 设置触发源
        status = GXSetEnum(this->g_device, GX_ENUM_TRIGGER_SOURCE, GX_TRIGGER_SOURCE_LINE2);
        // 设置曝光模式
        status = GXSetEnum(this->g_device, GX_ENUM_EXPOSURE_MODE, GX_EXPOSURE_MODE_TIMED);
        // 设置曝光时间
        status = GXSetFloat(this->g_device, GX_FLOAT_EXPOSURE_TIME, this->exposure_t);
        // 举例引脚选择为 Line3
        status = GXSetEnum(this->g_device, GX_ENUM_LINE_SELECTOR, GX_ENUM_LINE_SELECTOR_LINE3);
        // 设置引脚方向为输出
        status = GXSetEnum(this->g_device, GX_ENUM_LINE_MODE, GX_ENUM_LINE_MODE_OUTPUT);
        // 可选操作引脚电平反转
        status = GXSetBool(this->g_device, GX_BOOL_LINE_INVERTER, true);
        // 如果设置输出源为闪光灯,则如下代码
        status = GXSetEnum(this->g_device, GX_ENUM_LINE_SOURCE, GX_ENUM_LINE_SOURCE_STROBE);
        // 设置采集帧率
        if (control_mode == 0)
        {
            status = GXSetEnum(this->g_device, GX_ENUM_ACQUISITION_FRAME_RATE_MODE, GX_ACQUISITION_FRAME_RATE_MODE_ON);
            status = GXSetFloat(this->g_device, GX_FLOAT_ACQUISITION_FRAME_RATE, this->frequency);
        }
        // 申请图像数据空间
        cudaMallocManaged(&this->g_frame_data.pImgBuf, 2560 * 1024 * 8, cudaMemAttachHost);
        // 申请vpidata空间
        // cudaMallocManaged(&vpiData_left.buffer.pitch.planes[0].data, this->width * this->height * 8, cudaMemAttachHost);
        // cudaMallocManaged(&vpiData_right.buffer.pitch.planes[0].data, this->width * this->height * 8, cudaMemAttachHost);
        // cudaMallocManaged(&vpidata16.buffer.pitch.planes[0].data, this->width * this->height * 16, cudaMemAttachHost);
        // // 创建vpistream
        // CHECK_STATUS(vpiStreamCreate(VPI_BACKEND_CPU | VPI_BACKEND_CUDA, &stream));
        return status;
    }

    void *Gxdrive::ProcGetImage(void *pParam)
    {
        std::cout << "Enter acquire!" << std::endl;
        GX_STATUS status = GX_STATUS_SUCCESS;
        // 发送开采命令
        status = GXSendCommand(this->g_device, GX_COMMAND_ACQUISITION_START);
        if (status != GX_STATUS_SUCCESS)
        {
            GetErrorString(status);
        }
        while (ros::ok())
        {
            if (this->g_frame_data.pImgBuf == NULL)
            {
                LOG(WARNING) << "Image is NULL!";
                continue;
            }
            // auto start = std::chrono::system_clock::now();
            status = GXGetImage(this->g_device, &this->g_frame_data, 100);
            // auto end = std::chrono::system_clock::now();
            // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            // std::cout << "cost: "
            //           << double(duration.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den << "秒" << std::endl;
            if (status == GX_STATUS_SUCCESS)
            {
                // ROS_INFO("gx id:%ld",this->g_frame_data.nFrameID);
                int nWidth = this->width;
                int nHeight = this->height;
                if (this->g_frame_data.nStatus == 0)
                {
                    // ROS_INFO("gx time:%ld", this->g_frame_data.nTimestamp);
                    nppiCFAToRGB_8u_C1C3R((u_char *)this->g_frame_data.pImgBuf, 2560, {2560, 1024}, {0, 0, 2560, 1024},
                                          this->tmp_color, 2560 * 3,
                                          NPPI_BAYER_RGGB, NPPI_INTER_UNDEFINED);

                    nppiResize_8u_C3R(this->tmp_color, 2560 * 3, {2560, 1024}, {0, 0, 1280, 1024},
                                      this->left_rgb, nWidth * 3, {nWidth, nHeight}, {0, 0, nWidth, nHeight}, NPPI_INTER_CUBIC); // 三次插值
                    nppiResize_8u_C3R(this->tmp_color, 2560 * 3, {2560, 1024}, {1280, 0, 1280, 1024},
                                      this->right_rgb, nWidth * 3, {nWidth, nHeight}, {0, 0, nWidth, nHeight}, NPPI_INTER_CUBIC); // 三次插值

                    nppiGammaFwd_8u_C3R(this->left_rgb, nWidth * 3, this->left_gamma, nWidth * 3, {nWidth, nHeight});
                    nppiGammaFwd_8u_C3R(this->right_rgb, nWidth * 3, this->right_gamma, nWidth * 3, {nWidth, nHeight});

                    nppiRGBToGray_8u_C3C1R(this->left_gamma, nWidth * 3, this->left_gray, nWidth, {nWidth, nHeight});
                    nppiRGBToGray_8u_C3C1R(this->right_gamma, nWidth * 3, this->right_gray, nWidth, {nWidth, nHeight});

                    nppiRemap_8u_C1R(this->left_gray, {960, 768}, 960, {0, 0, 960, 768}, pLxmap, 3840, pLymap, 3840, left_stereo, 960, {960, 768}, NPPI_INTER_CUBIC);
                    nppiRemap_8u_C1R(this->right_gray, {960, 768}, 960, {0, 0, 960, 768}, pRxmap, 3840, pRymap, 3840, right_stereo, 960, {960, 768}, NPPI_INTER_CUBIC);
                    ROS_DEBUG("frame:%ld", this->frame);
                    cudaStreamSynchronize(NULL);

                    memcpy(reinterpret_cast<u_char *>(&this->img_left_gray.data[0]), this->left_stereo, nWidth * nHeight);
                    memcpy(reinterpret_cast<u_char *>(&this->img_right_gray.data[0]), this->right_stereo, nWidth * nHeight);
                    memcpy(reinterpret_cast<u_char *>(&this->img_left.data[0]), this->left_gamma, nWidth * nHeight * 3);
                    memcpy(reinterpret_cast<u_char *>(&this->img_right.data[0]), this->right_gamma, nWidth * nHeight * 3);
                    if (this->frame >= this->sync_frame)
                    {
                        uint64_t frame_id = this->frame - this->sync_frame;
                        ROS_DEBUG("image pro current_time:%lf", this->current_time.toSec());
                        this->time_image = this->current_time - ros::Duration(0, this->exposure_t * 500);

                        this->gframe_dura = this->g_frame_data.nTimestamp - this->last_gframe_time;
                        // ROS_INFO("%ld", this->gframe_dura);
                        // ROS_INFO("fpga_gx_dura_min:%ld", fpga_gx_dura_min);
                        if (fabs(this->gframe_dura / 1000000000.0 - this->pwm_period_s) > 0.001)
                        {
                            LOG(INFO) << "id:" << this->id_front << "is drop frame:" << this->dropframe;
                            this->dropframe = false;
                            LOG(WARNING) << "timestamp exception,last_time:" << this->last_time_image.toSec()
                                         << "current_time:" << this->time_image.toSec();
                            uint64_t true_time = this->last_time_image.toNSec() + this->gframe_dura;
                            this->time_image = ros::Time((uint32_t)(true_time / 1000000000.0), true_time % 1000000000);
                            // ROS_INFO("time_image:%lf", this->time_image.toSec());
                        }
                        if (this->last_time_image.toSec() - this->time_image.toSec() >= 0)
                        {
                            LOG(ERROR) << "old time>new time,old:" << this->last_time_image.toSec() << "new:" << this->time_image.toSec();
                            this->time_image = this->last_time_image + ros::Duration(0, this->pwm_period * 1000000);
                        }
                        this->img_left_gray.header.stamp = this->time_image;
                        this->img_left_gray.header.seq = frame_id;
                        // 右图灰度图
                        this->img_right_gray.header.stamp = this->time_image;
                        this->img_right_gray.header.seq = frame_id;
                        // 进行话题发布
                        this->pub_img_left_gray.publish(this->img_right_gray);
                        this->pub_img_right_gray.publish(this->img_left_gray);

                        // 左图
                        this->img_left.header.stamp = this->time_image;
                        // 右图
                        this->img_right.header.stamp = this->time_image;
                        // 上一次的时间戳
                        this->last_time_image = this->time_image;
                        this->last_gframe_time = this->g_frame_data.nTimestamp;
                        this->pub_img_left.publish(img_right);
                        this->pub_img_right.publish(img_left);
                    }
                    this->frame++;
                }
            }
            else
            {
                if (control_mode == 1)
                {
                    LOG(WARNING) << "cannot get image";
                    if (!ispre)
                    {
                        ros::shutdown();
                        break;
                    }
                    ispre = false;
                    this->tricon_client.call(this->trigger_control);
                }
            }
        }
        return 0;
    }

    int Gxdrive::UnPreForImage()
    {
        GX_STATUS status = GX_STATUS_SUCCESS;
        uint32_t ret = 0;
        LOG(INFO) << "exit Image!";
        // 发送停采命令
        status = GXSendCommand(this->g_device, GX_COMMAND_ACQUISITION_STOP);
        if (status != GX_STATUS_SUCCESS)
        {
            GetErrorString(status);
            return status;
        }
        ret = pthread_join(this->g_acquire_thread, NULL);
        if (ret != 0)
        {
            printf("<Failed to release resources>\n");
            return ret;
        }
        ret = pthread_join(this->depth_thread, NULL);
        if (ret != 0)
        {
            printf("<Failed to release resources>\n");
            return ret;
        }
        // 释放buffer
        if (this->g_frame_data.pImgBuf != NULL)
        {
            delete (this->g_frame_data.pImgBuf);
            this->g_frame_data.pImgBuf = NULL;
        }

        return 0;
    }

    void Gxdrive::GetErrorString(GX_STATUS error_status)
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

    void Gxdrive::Time_callback(const mavros_msgs::CamIMUStampConstPtr &cam_stamp)
    {
        if (this->id_front != cam_stamp->frame_seq_id - 1)
        {
            this->dropframe = true;
            LOG(WARNING) << "drop frame! id_front:" << this->id_front << "now:" << cam_stamp->frame_seq_id;
        }
        // ROS_INFO("feikong:%lf", ros::Time::now().toSec() - cam_stamp->frame_stamp.toSec());
        this->current_time = cam_stamp->frame_stamp; // 当前的时间戳
        this->id_front = cam_stamp->frame_seq_id;
    }

    void Gxdrive::initLog(const char *logfile)
    {
        google::InitGoogleLogging("Gxdrive");
        google::SetLogDestination(google::GLOG_INFO, logfile);
        google::SetStderrLogging(google::GLOG_WARNING);
        google::SetLogFilenameExtension("log_");
        FLAGS_colorlogtostderr = true;          // Set log color
        FLAGS_logbufsecs = 0;                   // Set log output speed(s)
        FLAGS_max_log_size = 1024;              // Set max log file size
        FLAGS_stop_logging_if_full_disk = true; // If disk is full
    }