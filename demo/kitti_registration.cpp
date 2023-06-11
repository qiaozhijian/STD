//
// Created by qzj on 23-6-12.
//
#include "include/STDesc.h"
#include <nav_msgs/Odometry.h>

pcl::PointCloud<pcl::PointXYZI>::Ptr read_lidar_data(const std::string lidar_data_path, ConfigSetting config_setting);
void LoadKittiPose(std::string pose_file, std::string calib_file, std::vector<Eigen::Matrix4d>& poses);
ConfigSetting GetKittiConfig();

int main(int argc, char **argv) {

    std::string lidar_path = "/media/qzj/Document/datasets/KITTI/odometry/data_odometry_velodyne/dataset/sequences/00/velodyne/";
    std::string pose_path = "/media/qzj/Document/datasets/KITTI/odometry/data_odometry_velodyne/dataset/sequences/00/poses.txt";
    std::string calib_path = "/media/qzj/Document/datasets/KITTI/odometry/data_odometry_velodyne/dataset/sequences/00/calib.txt";
    int src_idx = 0, dst_idx = 4405;
    ConfigSetting config_setting = GetKittiConfig();

    std::stringstream lidar_data_path;
    lidar_data_path << lidar_path << std::setfill('0') << std::setw(6) << src_idx << ".bin";
    pcl::PointCloud<pcl::PointXYZI>::Ptr src_data = read_lidar_data(lidar_data_path.str(), config_setting);
    lidar_data_path.str("");
    lidar_data_path << lidar_path << std::setfill('0') << std::setw(6) << dst_idx << ".bin";
    pcl::PointCloud<pcl::PointXYZI>::Ptr dst_data = read_lidar_data(lidar_data_path.str(), config_setting);

    std::vector<Eigen::Matrix4d> poses;
    LoadKittiPose(pose_path, calib_path, poses);
    std::cout << "Sucessfully load pose with number: " << poses.size() << std::endl;

    Eigen::Matrix4d tf = poses[dst_idx].inverse() * poses[src_idx];

    STDescManager *std_manager = new STDescManager(config_setting);
    auto t1 = std::chrono::high_resolution_clock::now();
    std::vector<STDesc> stds_vec;
    std_manager->GenerateSTDescs(src_data, stds_vec);


}

ConfigSetting GetKittiConfig(){
    ConfigSetting config_setting;

    config_setting.ds_size_ = 0.25;
    config_setting.maximum_corner_num_ = 100;

    config_setting.plane_detection_thre_ = 0.01;
    config_setting.plane_merge_normal_thre_ = 0.2;
    config_setting.voxel_size_ = 2.0;
    config_setting.voxel_init_num_ = 10;
    config_setting.proj_image_resolution_ = 0.5;
    config_setting.proj_dis_min_ = 0;
    config_setting.proj_dis_max_ = 5;
    config_setting.corner_thre_ = 10;

    config_setting.descriptor_near_num_ = 10;
    config_setting.descriptor_min_len_ = 2;
    config_setting.descriptor_max_len_ = 50;
    config_setting.non_max_suppression_radius_ = 2;
    config_setting.std_side_resolution_ = 0.2;

    config_setting.skip_near_num_ = 50;
    config_setting.candidate_num_ = 50;
    config_setting.sub_frame_num_ = 10;
    config_setting.vertex_diff_threshold_ = 0.5;
    config_setting.rough_dis_threshold_ = 0.01;
    config_setting.normal_threshold_ = 0.2;
    config_setting.dis_threshold_ = 0.5;
    config_setting.icp_threshold_ = 0.4;

    return config_setting;
}

Eigen::Matrix4d GetTr(const std::string& calib_file) {
    std::fstream f;
    f.open(calib_file, std::ios::in);
    if (!f.is_open()) {
        std::cerr << "Cannot open calib file: " << calib_file << std::endl;
    }
    std::string line;
    Eigen::Matrix4d Tr = Eigen::Matrix4d::Identity();
    while (std::getline(f, line)) {
        std::stringstream ss(line);
        std::string tag;
        ss >> tag;
        if (tag == "Tr:") {
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 4; ++j) {
                    ss >> Tr(i, j);
                }
            }
        }
    }
    return Tr;
}

void LoadKittiPose(std::string pose_file, std::string calib_file, std::vector<Eigen::Matrix4d>& poses) {
    //    read kitti pose txt
    std::fstream f;
    f.open(pose_file, std::ios::in);
    if (!f.is_open()) {
        LOG(FATAL) << "Cannot open pose file: " << pose_file;
    }
    Eigen::Matrix4d Tr = GetTr(calib_file);
    std::string line;
    while (std::getline(f, line)) {
        std::stringstream ss(line);
        Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 4; ++j) {
                ss >> Twc(i, j);
            }
        }
        Eigen::Matrix4d Twl = Twc * Tr;
        poses.push_back(Twl);
    }
}

// Read KITTI data
pcl::PointCloud<pcl::PointXYZI>::Ptr read_lidar_data(const std::string lidar_data_path, ConfigSetting config_setting) {
    std::ifstream lidar_data_file;
    lidar_data_file.open(lidar_data_path,
                         std::ifstream::in | std::ifstream::binary);
    if (!lidar_data_file) {
        std::cout << "Read End..." << std::endl;
        std::vector<float> nan_data;
        return pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    }
    lidar_data_file.seekg(0, std::ios::end);
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
    lidar_data_file.seekg(0, std::ios::beg);

    std::vector<float> lidar_data_buffer(num_elements);
    lidar_data_file.read(reinterpret_cast<char *>(&lidar_data_buffer[0]),
                         num_elements * sizeof(float));

    pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    for (std::size_t i = 0; i < lidar_data_buffer.size(); i += 4) {
        pcl::PointXYZI point;
        point.x = lidar_data_buffer[i];
        point.y = lidar_data_buffer[i + 1];
        point.z = lidar_data_buffer[i + 2];
        point.intensity = lidar_data_buffer[i + 3];
        current_cloud->push_back(point);
    }
    down_sampling_voxel(*current_cloud, config_setting.ds_size_);

    return current_cloud;
}