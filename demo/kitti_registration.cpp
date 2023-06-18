//
// Created by qzj on 23-6-12.
//
#include "include/STDesc.h"
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

pcl::PointCloud<pcl::PointXYZI>::Ptr read_lidar_data(const std::string &lidar_data_path, ConfigSetting config_setting);

void LoadKittiPose(const std::string &pose_file, const std::string &calib_file, std::vector<Eigen::Matrix4d> &poses);

ConfigSetting GetKittiConfig();

Eigen::Isometry3d
estimateTF(pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_src, pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_tgt,
           ConfigSetting &config_setting) {
    Eigen::Isometry3d T_ts; // T_tgt = T_est * T_src, T_est = T_ts
    T_ts.setIdentity();

    std::unique_ptr<STDescManager> std_manager(new STDescManager(config_setting));

    std::vector<STDesc> stds_vec_src, stds_vec_tgt;

    std_manager->GenerateSTDescs(ptr_tgt, stds_vec_tgt);
    std_manager->AddSTDescs(stds_vec_tgt);
    std_manager->key_cloud_vec_.push_back(ptr_tgt);

    std_manager->GenerateSTDescs(ptr_src, stds_vec_src);

    std::pair<int, double> search_result(-1, 0);
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform;
    loop_transform.first << 0, 0, 0;
    loop_transform.second = Eigen::Matrix3d::Identity();
    std::vector<std::pair<STDesc, STDesc>> loop_std_pair;  // For visualization

    std_manager->SearchLoop(stds_vec_src, search_result, loop_transform, loop_std_pair);

    printf("#loop_std_pair: %lu\n", loop_std_pair.size());

    if (search_result.first >= 0) {
        std::cout << "[Loop Detection] triggle loop: , score:" << search_result.second << std::endl;
        T_ts.rotate(Eigen::Quaterniond(loop_transform.second));
        T_ts.pretranslate(loop_transform.first);
    } else {
        printf("No loops detected.\n");
    }
    return T_ts;

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rosbag_play_test");
    ros::NodeHandle nh;

    // rosparam load $(rospack find std_detector)/config/config_kitti_reg.yaml
    ConfigSetting config_setting;
    read_parameters(nh, config_setting);
    printf("ds_size: %f\n", config_setting.ds_size_);

    std::string lidar_path = "/media/qzj/Document/datasets/KITTI/odometry/data_odometry_velodyne/dataset/sequences/00/velodyne/";
    std::string pose_path = "/media/qzj/Document/datasets/KITTI/odometry/data_odometry_velodyne/dataset/sequences/00/poses.txt";
    std::string calib_path = "/media/qzj/Document/datasets/KITTI/odometry/data_odometry_velodyne/dataset/sequences/00/calib.txt";
    int src_idx = 0, dst_idx = 4405;
    ConfigSetting config_setting = GetKittiConfig();

//    int src_idx = 4405, dst_idx = 0;
    int src_idx = 7, dst_idx = 0;
    std::stringstream lidar_data_path;
    lidar_data_path << lidar_path << std::setfill('0') << std::setw(6) << src_idx << ".bin";
    pcl::PointCloud<pcl::PointXYZI>::Ptr src_data = read_lidar_data(lidar_data_path.str(), config_setting);
    lidar_data_path.str("");
    lidar_data_path << lidar_path << std::setfill('0') << std::setw(6) << dst_idx << ".bin";
    pcl::PointCloud<pcl::PointXYZI>::Ptr dst_data = read_lidar_data(lidar_data_path.str(), config_setting);

    std::vector<Eigen::Matrix4d> poses;
    LoadKittiPose(pose_path, calib_path, poses);
    std::cout << "Sucessfully load pose with number: " << poses.size() << std::endl;

    Eigen::Matrix4d T_ts_gt = poses[dst_idx].inverse() * poses[src_idx];

    std::cout << "T_src:\n" << poses[src_idx] << std::endl;
    std::cout << "T_tgt:\n" << poses[dst_idx] << std::endl;

    std::cout << "True tf:\n" << T_ts_gt.matrix() << std::endl;
    Eigen::Isometry3d Tf_est = estimateTF(src_data, dst_data, config_setting);
    std::cout << "Est tf:\n" << Tf_est.matrix() << std::endl;
    std::cout << "Est error:\n" << T_ts_gt * (Tf_est.matrix().inverse()) << std::endl;

    // a. The pc aligned with GT transform in tgt/dst frame:
//    pcl::transformPointCloud(*src_data, *src_data, T_ts_gt.cast<float>());

    // b. The pc aligned with estimated transform.
    pcl::transformPointCloud(*src_data, *src_data, Tf_est.cast<float>());

    ros::Publisher pub_src = nh.advertise<sensor_msgs::PointCloud2>("/kitti_reg_src_tf", 1000);
    ros::Publisher pub_tgt = nh.advertise<sensor_msgs::PointCloud2>("/kitti_reg_tgt", 1000);
    ros::Publisher pubSTD = nh.advertise<visualization_msgs::MarkerArray>("descriptor_line", 10);
    sensor_msgs::PointCloud2 kitti_reg_src_tf, kitti_reg_tgt;

    pcl::toROSMsg(*src_data, kitti_reg_src_tf);
    pcl::toROSMsg(*dst_data, kitti_reg_tgt);
    kitti_reg_src_tf.header.frame_id = "world";
    kitti_reg_tgt.header.frame_id = "world";

    ros::Rate rate(1);
    int cnt = 0;

    // rviz -d $(rospack find std_detector)/rviz_cfg/viz_kitti_reg.rviz
    while (ros::ok()) {
        printf("Publish %d\n", cnt++);
        kitti_reg_src_tf.header.stamp = ros::Time::now();
        kitti_reg_tgt.header.stamp = ros::Time::now();
        pub_src.publish(kitti_reg_src_tf);
        pub_tgt.publish(kitti_reg_tgt);
//        publish_std_pairs(loop_std_pair, pubSTD);
        rate.sleep();
    }


    return 0;
}

ConfigSetting GetKittiConfig() {
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

Eigen::Matrix4d GetTr(const std::string &calib_file) {
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

void LoadKittiPose(const std::string &pose_file, const std::string &calib_file, std::vector<Eigen::Matrix4d> &poses) {
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
        Eigen::Matrix4d T_lcam0_lcam = Eigen::Matrix4d::Identity();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 4; ++j) {
                ss >> T_lcam0_lcam(i, j);
            }
        }
        // Tr is T_leftcam_velod_
        // T_w_velod = np.linalg.inv(T_leftcam_velod_) @ tmp_T_lc0_lc @ T_leftcam_velod_
        Eigen::Matrix4d Twl = Tr.inverse() * T_lcam0_lcam * Tr;
        poses.push_back(Twl);
    }
}

// Read KITTI data
pcl::PointCloud<pcl::PointXYZI>::Ptr read_lidar_data(const std::string &lidar_data_path, ConfigSetting config_setting) {
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