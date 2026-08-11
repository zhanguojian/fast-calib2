/* 
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "qr_detect.hpp"
#include "lidar_detect.hpp"
#include "data_preprocess.hpp"

#include <iomanip>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    // 允许从 launch/yaml 自动声明参数覆盖值
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<rclcpp::Node>("mono_qr_pattern", options);

    // 读取参数
    Params params = loadParameters(node);
    std::string mounting_error;
    if (!validateLidarMountAxes(params.lidar_forward_axis, params.lidar_up_axis,
                                mounting_error))
    {
      ROS_ERROR_STREAM("[Main] Invalid LiDAR mounting-axis configuration: "
                       << mounting_error);
      rclcpp::shutdown();
      return 1;
    }

    std::string output_error;
    if (!ensureDirectoryTree(params.output_path, output_error))
    {
      ROS_ERROR_STREAM("[Main] Invalid output directory: " << output_error);
      rclcpp::shutdown();
      return 1;
    }

    // Load the inputs before constructing detectors so camera calibration can
    // be checked against the actual image resolution.
    DataPreprocessPtr dataPreprocessPtr;
    dataPreprocessPtr.reset(new DataPreprocess(params));

    // 读取图像和点云
    cv::Mat img_input = dataPreprocessPtr->img_input_;
    pcl::PointCloud<Common::Point>::Ptr cloud_input = dataPreprocessPtr->cloud_input_;
    if (img_input.empty())
    {
      ROS_ERROR_STREAM("[Main] Image is empty. Check image_path: " << params.image_path);
      rclcpp::shutdown();
      return 1;
    }
    if (!cloud_input || cloud_input->empty())
    {
      ROS_ERROR_STREAM("[Main] Point cloud is empty. Check bag_path and lidar_topic: "
                       << params.bag_path << ", " << params.lidar_topic);
      rclcpp::shutdown();
      return 1;
    }
    if (dataPreprocessPtr->lidar_type_ == LiDARType::Unknown)
    {
      ROS_ERROR_STREAM("[Main] Unknown LiDAR message type. Check lidar_topic: "
                       << params.lidar_topic);
      rclcpp::shutdown();
      return 1;
    }

    std::string camera_error;
    if (!validateCameraCalibrationForImage(params, img_input.cols, img_input.rows,
                                           camera_error))
    {
      ROS_ERROR_STREAM("[Main] Invalid camera calibration: " << camera_error);
      rclcpp::shutdown();
      return 1;
    }

    // 初始化 QR 检测和 LiDAR 检测
    QRDetectPtr qrDetectPtr;
    qrDetectPtr.reset(new QRDetect(node, params));

    LidarDetectPtr lidarDetectPtr;
    lidarDetectPtr.reset(new LidarDetect(node, params));
    
    // 检测 QR 码
    PointCloud<PointXYZ>::Ptr qr_center_cloud(new PointCloud<PointXYZ>);
    qr_center_cloud->reserve(4);
    qrDetectPtr->detect_qr(img_input, qr_center_cloud);
    if (qr_center_cloud->size() != TARGET_NUM_CIRCLES)
    {
      ROS_ERROR_STREAM("[Main] Expected " << TARGET_NUM_CIRCLES
                       << " camera target centers, got " << qr_center_cloud->size() << ".");
      rclcpp::shutdown();
      return 1;
    }

    // 检测 LiDAR 数据
    PointCloud<PointXYZ>::Ptr lidar_center_cloud(new PointCloud<PointXYZ>);
    lidar_center_cloud->reserve(4);
    
    switch (dataPreprocessPtr->lidar_type_)
    {
        case LiDARType::Solid:
            lidarDetectPtr->detect_solid_lidar(cloud_input, lidar_center_cloud);
            break;

        case LiDARType::Mech:
            lidarDetectPtr->detect_mech_lidar(cloud_input, lidar_center_cloud);
            break;

        default:
            std::cerr << BOLDYELLOW 
                    << "[Main] Unknown LiDAR type." 
                    << RESET << std::endl;
            break;
    }
    if (lidar_center_cloud->size() != TARGET_NUM_CIRCLES)
    {
      ROS_ERROR_STREAM("[Main] Expected " << TARGET_NUM_CIRCLES
                       << " LiDAR target centers, got " << lidar_center_cloud->size() << ".");
      rclcpp::shutdown();
      return 1;
    }

    // 对 QR 和 LiDAR 检测到的圆心进行排序
    PointCloud<PointXYZ>::Ptr qr_centers(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr lidar_centers(new PointCloud<PointXYZ>);
    if (!sortPatternCenters(qr_center_cloud, qr_centers, "camera") ||
        !sortPatternCenters(lidar_center_cloud, lidar_centers, "lidar",
                            params.lidar_forward_axis, params.lidar_up_axis))
    {
      ROS_ERROR("[Main] Failed to sort target centers. Check the LiDAR mounting-axis configuration.");
      rclcpp::shutdown();
      return 1;
    }

    validateTargetGeometry(qr_centers, params.delta_width_circles, params.delta_height_circles, "QR");
    validateTargetGeometry(lidar_centers, params.delta_width_circles, params.delta_height_circles, "LiDAR");

    // 保存中间结果：排序后的 LiDAR 圆心和 QR 圆心
    saveTargetHoleCenters(lidar_centers, qr_centers, params);

    // 计算外参
    Eigen::Matrix4f transformation;
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd;
    svd.estimateRigidTransformation(*lidar_centers, *qr_centers, transformation);

    // 将 LiDAR 点云转换到 QR 码坐标系
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_lidar_centers(new pcl::PointCloud<pcl::PointXYZ>);
    aligned_lidar_centers->reserve(lidar_centers->size());
    alignPointCloud(lidar_centers, aligned_lidar_centers, transformation);
    
    double rmse = computeRMSE(qr_centers, aligned_lidar_centers);
    if (rmse > 0) 
    {
      std::cout << BOLDYELLOW << "[Result] RMSE: " << BOLDRED << std::fixed << std::setprecision(4)
      << rmse << " m" << RESET << std::endl;
    }

    std::cout << BOLDYELLOW << "[Result] Single-scene calibration: extrinsic parameters T_cam_lidar = " << RESET << std::endl;
    std::cout << BOLDCYAN << std::fixed << std::setprecision(6) << transformation << RESET << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    projectPointCloudToImage(cloud_input, transformation, qrDetectPtr->cameraMatrix_, qrDetectPtr->distCoeffs_, img_input, colored_cloud);

    saveCalibrationResults(params, transformation, colored_cloud, qrDetectPtr->imageCopy_);

    auto colored_cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("colored_cloud", 1);
    auto aligned_lidar_centers_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("aligned_lidar_centers", 1);

    // 主循环：持续发布调试点云供 RViz2 查看
    rclcpp::Rate rate(1.0);
    while (rclcpp::ok()) 
    {
      if (DEBUG) 
      {
        // 发布 QR 检测结果
        sensor_msgs::msg::PointCloud2 qr_centers_msg;
        pcl::toROSMsg(*qr_centers, qr_centers_msg);
        qr_centers_msg.header.stamp = node->now();
        qr_centers_msg.header.frame_id = "map";
        qrDetectPtr->qr_pub_->publish(qr_centers_msg);

        // 发布 LiDAR 检测结果
        sensor_msgs::msg::PointCloud2 lidar_centers_msg;
        pcl::toROSMsg(*lidar_centers, lidar_centers_msg);
        lidar_centers_msg.header = qr_centers_msg.header;
        lidarDetectPtr->center_pub_->publish(lidar_centers_msg);

        // 发布中间结果
        sensor_msgs::msg::PointCloud2 filtered_cloud_msg;
        pcl::toROSMsg(*lidarDetectPtr->getFilteredCloud(), filtered_cloud_msg);
        filtered_cloud_msg.header = qr_centers_msg.header;
        lidarDetectPtr->filtered_pub_->publish(filtered_cloud_msg);

        sensor_msgs::msg::PointCloud2 plane_cloud_msg;
        pcl::toROSMsg(*lidarDetectPtr->getPlaneCloud(), plane_cloud_msg);
        plane_cloud_msg.header = qr_centers_msg.header;
        lidarDetectPtr->plane_pub_->publish(plane_cloud_msg);

        sensor_msgs::msg::PointCloud2 annulus_cloud_msg;
        pcl::toROSMsg(*lidarDetectPtr->getAnnulusOriginalCloud(), annulus_cloud_msg);
        annulus_cloud_msg.header = qr_centers_msg.header;
        lidarDetectPtr->annulus_pub_->publish(annulus_cloud_msg);

        sensor_msgs::msg::PointCloud2 boundary_cloud_msg;
        pcl::toROSMsg(*lidarDetectPtr->getBoundaryOriginalCloud(), boundary_cloud_msg);
        boundary_cloud_msg.header = qr_centers_msg.header;
        lidarDetectPtr->boundary_pub_->publish(boundary_cloud_msg);

        sensor_msgs::msg::PointCloud2 aligned_cloud_msg;
        pcl::toROSMsg(*lidarDetectPtr->getAlignedCloud(), aligned_cloud_msg);
        aligned_cloud_msg.header = qr_centers_msg.header;
        lidarDetectPtr->aligned_pub_->publish(aligned_cloud_msg);

        sensor_msgs::msg::PointCloud2 edge_cloud_msg;
        pcl::toROSMsg(*lidarDetectPtr->getEdgeCloud(), edge_cloud_msg);
        edge_cloud_msg.header = qr_centers_msg.header;
        lidarDetectPtr->edge_pub_->publish(edge_cloud_msg);

        sensor_msgs::msg::PointCloud2 lidar_centers_z0_msg;
        pcl::toROSMsg(*lidarDetectPtr->getCenterZ0Cloud(), lidar_centers_z0_msg);
        lidar_centers_z0_msg.header = qr_centers_msg.header;
        lidarDetectPtr->center_z0_pub_->publish(lidar_centers_z0_msg);

        // 发布外参变换后的 LiDAR 圆心
        sensor_msgs::msg::PointCloud2 aligned_lidar_centers_msg;
        pcl::toROSMsg(*aligned_lidar_centers, aligned_lidar_centers_msg);
        aligned_lidar_centers_msg.header = qr_centers_msg.header;
        aligned_lidar_centers_pub->publish(aligned_lidar_centers_msg);

        // 发布彩色点云
        sensor_msgs::msg::PointCloud2 colored_cloud_msg;
        pcl::toROSMsg(*colored_cloud, colored_cloud_msg);
        colored_cloud_msg.header = qr_centers_msg.header;
        colored_cloud_pub->publish(colored_cloud_msg);

        // cv::imshow("result", qrDetectPtr->imageCopy_);
      }
      // cv::waitKey(1);
      rclcpp::spin_some(node);
      rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
