/* 
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef DATA_PREPROCESS_HPP
#define DATA_PREPROCESS_HPP

#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <fast_calib/msg/custom_msg.hpp>

#include <fstream>
#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "common_lib.h"

using namespace std;

enum class LiDARType : int {
    Unknown = 0,
    Solid   = 1,   // 固态（如 Livox）
    Mech    = 2    // 机械式多线
};

namespace fast_calib_bag
{
// 判断消息类型是否为 Livox CustomMsg（兼容 driver / driver2 / 本包）
inline bool isLivoxCustomMsgType(const std::string &type)
{
    return type.find("CustomMsg") != std::string::npos;
}

inline bool isPointCloud2Type(const std::string &type)
{
    return type.find("PointCloud2") != std::string::npos;
}

// 规范化 topic：去掉末尾斜杠后比较
inline std::string normalizeTopic(std::string topic)
{
    while (!topic.empty() && topic.back() == '/' && topic.size() > 1) {
        topic.pop_back();
    }
    return topic;
}

inline bool topicMatches(const std::string &bag_topic, const std::string &wanted)
{
    const std::string a = normalizeTopic(bag_topic);
    const std::string b = normalizeTopic(wanted);
    return a == b || a == ("/" + b) || ("/" + a) == b;
}

inline void appendLivoxCustomMsg(const fast_calib::msg::CustomMsg &livox_custom_msg,
                          std::uint32_t current_scan_id,
                          pcl::PointCloud<Common::Point>::Ptr cloud_input)
{
    cloud_input->reserve(cloud_input->size() + livox_custom_msg.point_num);
    for (uint32_t i = 0; i < livox_custom_msg.point_num; ++i)
    {
        Common::Point p;
        p.x = livox_custom_msg.points[i].x;
        p.y = livox_custom_msg.points[i].y;
        p.z = livox_custom_msg.points[i].z;
        p.intensity = static_cast<float>(livox_custom_msg.points[i].reflectivity);
        // Livox CustomPoint 的 line 字段表示线号
        p.ring = static_cast<std::uint16_t>(livox_custom_msg.points[i].line);
        p.scan_id = current_scan_id;
        cloud_input->push_back(p);
    }
}

inline void appendPointCloud2(const sensor_msgs::msg::PointCloud2 &pcl_msg,
                       std::uint32_t current_scan_id,
                       pcl::PointCloud<Common::Point>::Ptr cloud_input,
                       LiDARType &lidar_type)
{
    bool has_ring = false;
    bool has_intensity = false;
    bool has_reflectivity = false;
    for (const auto &f : pcl_msg.fields)
    {
        if (f.name == "ring") has_ring = true;
        if (f.name == "intensity") has_intensity = true;
        if (f.name == "reflectivity") has_reflectivity = true;
    }

    sensor_msgs::PointCloud2ConstIterator<float> it_x(pcl_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(pcl_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(pcl_msg, "z");

    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<std::uint16_t>> it_ring_ptr;
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> it_intensity_ptr;
    if (has_ring)
    {
        it_ring_ptr.reset(new sensor_msgs::PointCloud2ConstIterator<std::uint16_t>(pcl_msg, "ring"));
        lidar_type = LiDARType::Mech;
    }
    else
    {
        lidar_type = LiDARType::Solid;
    }
    if (has_intensity)
    {
        it_intensity_ptr.reset(new sensor_msgs::PointCloud2ConstIterator<float>(pcl_msg, "intensity"));
    }
    else if (has_reflectivity)
    {
        it_intensity_ptr.reset(new sensor_msgs::PointCloud2ConstIterator<float>(pcl_msg, "reflectivity"));
    }

    const size_t n = static_cast<size_t>(pcl_msg.width) * pcl_msg.height;
    cloud_input->reserve(cloud_input->size() + n);

    for (size_t i = 0; i < n; ++i, ++it_x, ++it_y, ++it_z)
    {
        Common::Point p;
        p.x = *it_x;
        p.y = *it_y;
        p.z = *it_z;

        if (has_ring)
        {
            p.ring = **it_ring_ptr;
            ++(*it_ring_ptr);
        }
        else
        {
            p.ring = 0xFFFF; // 未知线号
        }
        if (it_intensity_ptr)
        {
            p.intensity = **it_intensity_ptr;
            ++(*it_intensity_ptr);
        }
        else
        {
            p.intensity = 0.0f;
        }
        p.scan_id = current_scan_id;
        cloud_input->push_back(p);
    }
}
}  // namespace fast_calib_bag

class DataPreprocess
{
public:
    // 改成带线号的点云
    pcl::PointCloud<Common::Point>::Ptr cloud_input_;
    cv::Mat img_input_;
    LiDARType lidar_type_{LiDARType::Unknown};
    LiDARType lidarType() const { return lidar_type_; }

    DataPreprocess(Params &params)
        : cloud_input_(new pcl::PointCloud<Common::Point>)
    {
        string bag_path   = params.bag_path;
        string image_path = params.image_path;
        string lidar_topic = params.lidar_topic;

        // 读图像
        img_input_ = cv::imread(image_path, cv::IMREAD_UNCHANGED);
        if (img_input_.empty())
        {
            std::string msg = "Loading the image " + image_path + " failed";
            ROS_ERROR_STREAM(msg.c_str());
            return;
        }

        // ROS1 .bag 需先转换为 rosbag2
        namespace fs = std::filesystem;
        if (fs::exists(bag_path) && fs::is_regular_file(bag_path))
        {
            const auto ext = fs::path(bag_path).extension().string();
            if (ext == ".bag")
            {
                ROS_ERROR_STREAM(
                    "ROS1 .bag is not supported directly. Convert first, e.g.\n"
                    "  pip3 install rosbags\n"
                    "  rosbags-convert --src " << bag_path << " --dst " << bag_path << "_ros2\n"
                    "Then set bag_path to the converted rosbag2 directory.");
                return;
            }
        }

        if (!fs::exists(bag_path))
        {
            std::string msg = "Loading the rosbag " + bag_path + " failed (path not found)";
            ROS_ERROR_STREAM(msg.c_str());
            return;
        }
        ROS_INFO("Loading the rosbag %s", bag_path.c_str());

        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = bag_path;
        // 空字符串让 rosbag2 自动探测存储格式（sqlite3 / mcap 等）
        storage_options.storage_id = "";

        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";

        rosbag2_cpp::readers::SequentialReader reader;
        try {
            reader.open(storage_options, converter_options);
        } catch (const std::exception &e) {
            ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
            return;
        }

        // 建立 topic -> type 映射
        std::unordered_map<std::string, std::string> topic_types;
        for (const auto &topic_meta : reader.get_all_topics_and_types())
        {
            topic_types[topic_meta.name] = topic_meta.type;
        }

        rclcpp::Serialization<fast_calib::msg::CustomMsg> livox_serialization;
        rclcpp::Serialization<sensor_msgs::msg::PointCloud2> cloud_serialization;
        std::uint32_t scan_id = 0;

        while (reader.has_next())
        {
            auto bag_message = reader.read_next();
            if (!fast_calib_bag::topicMatches(bag_message->topic_name, lidar_topic))
            {
                continue;
            }

            const auto type_it = topic_types.find(bag_message->topic_name);
            if (type_it == topic_types.end())
            {
                continue;
            }
            const std::string &msg_type = type_it->second;

            rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);

            if (fast_calib_bag::isLivoxCustomMsgType(msg_type))
            {
                fast_calib::msg::CustomMsg livox_custom_msg;
                try {
                    livox_serialization.deserialize_message(&serialized_msg, &livox_custom_msg);
                } catch (const std::exception &e) {
                    ROS_WARN_STREAM("Failed to deserialize CustomMsg: " << e.what());
                    continue;
                }
                const std::uint32_t current_scan_id = scan_id++;
                lidar_type_ = LiDARType::Solid;
                fast_calib_bag::appendLivoxCustomMsg(livox_custom_msg, current_scan_id, cloud_input_);
                continue;
            }

            if (fast_calib_bag::isPointCloud2Type(msg_type))
            {
                sensor_msgs::msg::PointCloud2 pcl_msg;
                try {
                    cloud_serialization.deserialize_message(&serialized_msg, &pcl_msg);
                } catch (const std::exception &e) {
                    ROS_WARN_STREAM("Failed to deserialize PointCloud2: " << e.what());
                    continue;
                }
                const std::uint32_t current_scan_id = scan_id++;
                fast_calib_bag::appendPointCloud2(pcl_msg, current_scan_id, cloud_input_, lidar_type_);
                continue;
            }
        }

        ROS_INFO("Loaded %zu points from the rosbag.", cloud_input_->size());
    }
};

typedef std::shared_ptr<DataPreprocess> DataPreprocessPtr;

#endif // DATA_PREPROCESS_HPP
