#include <yaml-cpp/yaml.h>
#include <iostream>

double left_intrinsics[9], right_intrinsics[9];
double right_distortion[4], left_distortion[4];
double exposure_t, gamma_ratio, bal_r, bal_g, bal_b;
int control_mode, frequency, resolution;
std::string camera_style;

void yaml_read(std::string file_path)
{

    YAML::Node yaml_node = YAML::LoadFile(file_path);
    camera_style = yaml_node["Camera_style"].as<std::string>();
    control_mode = yaml_node["Control_mode"].as<int>();
    frequency = yaml_node["Frequency"].as<int>();
    resolution = yaml_node["Resolution"].as<int>();
    gamma_ratio = yaml_node["Gamma"].as<double>();
    exposure_t = yaml_node["Exposure_T"].as<double>();
    bal_r = yaml_node["Balance_ratio"]["r"].as<double>();
    bal_g = yaml_node["Balance_ratio"]["g"].as<double>();
    bal_b = yaml_node["Balance_ratio"]["b"].as<double>();
    for (int i = 0; i < 9; i++)
    {
        left_intrinsics[i] = yaml_node["Left_cam"]["intrinsics"][i].as<double>();
        right_intrinsics[i] = yaml_node["Right_cam"]["intrinsics"][i].as<double>();
    }
    for (int i = 0; i < 4; i++)
    {
        left_distortion[i] = yaml_node["Left_cam"]["distortion"][i].as<double>();
        right_distortion[i] = yaml_node["Right_cam"]["distortion"][i].as<double>();
    }
}

int main()
{
    std::string file_path = "/home/oem/Vscode/work/cam/camera.yaml";
    yaml_read(file_path);
    std::cout << "Camera_style:" << camera_style << std::endl;
    std::cout << "control_mode:" << control_mode << std::endl;
    std::cout << "frequency:" << frequency << std::endl;
    std::cout << "resolution:" << resolution << std::endl;
    std::cout << "gamma:" << gamma_ratio << std::endl;
    std::cout << "exposure_t:" << exposure_t << std::endl;
    std::cout << "bal_r:" << bal_r << std::endl;
    std::cout << "bal_g:" << bal_g << std::endl;
    std::cout << "bal_b:" << bal_b << std::endl;
    std::cout << "left_intrinsics:" << std::endl;
    for (int i = 0; i < 9; i++)
    {
        std::cout << left_intrinsics[i] << "\t";
    }
    std::cout << std::endl
              << "right_intrinsics:" << std::endl;
    for (int i = 0; i < 9; i++)
    {
        std::cout << right_intrinsics[i] << "\t";
    }
    std::cout << std::endl
              << "left_distortion:" << std::endl;
    for (int i = 0; i < 4; i++)
    {
        std::cout << left_distortion[i] << "\t";
    }
    std::cout << std::endl
              << "right_distortion:" << std::endl;
    for (int i = 0; i < 4; i++)
    {
        std::cout << right_distortion[i] << "\t";
    }
    std::cout << std::endl;
    return 0;
}