#include <gtest/gtest.h>

#include <array>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

#include "common_lib.h"

namespace
{
pcl::PointXYZ makePoint(const Eigen::Vector3f& p)
{
  pcl::PointXYZ out;
  out.x = p.x();
  out.y = p.y();
  out.z = p.z();
  return out;
}

Eigen::Vector3f toOptical(const Eigen::Vector3f& native,
                         const Eigen::Vector3f& forward,
                         const Eigen::Vector3f& left,
                         const Eigen::Vector3f& up)
{
  return Eigen::Vector3f(-native.dot(left),
                         -native.dot(up),
                          native.dot(forward));
}
}  // namespace

TEST(LidarMountAxes, ResolvesAll24RightHandedCombinations)
{
  const std::vector<std::string> axes = {"+x", "-x", "+y", "-y", "+z", "-z"};
  const std::array<Eigen::Vector3f, 4> body_points = {{
      Eigen::Vector3f(3.0f,  0.25f,  0.20f),
      Eigen::Vector3f(3.0f, -0.25f, -0.20f),
      Eigen::Vector3f(3.0f,  0.25f, -0.20f),
      Eigen::Vector3f(3.0f, -0.25f,  0.20f)}};

  int valid_combinations = 0;
  for (const auto& forward_name : axes)
  {
    for (const auto& up_name : axes)
    {
      Eigen::Vector3f forward;
      Eigen::Vector3f left;
      Eigen::Vector3f up;
      std::string normalized_forward;
      std::string normalized_left;
      std::string normalized_up;
      std::string error;
      if (!resolveLidarMountAxes(forward_name, up_name,
                                 forward, left, up,
                                 normalized_forward, normalized_left, normalized_up,
                                 error))
      {
        continue;
      }

      ++valid_combinations;
      EXPECT_NEAR(forward.dot(up), 0.0f, 1e-6f);
      EXPECT_NEAR((forward.cross(left) - up).norm(), 0.0f, 1e-6f);

      pcl::PointCloud<pcl::PointXYZ>::Ptr camera_points(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_points(new pcl::PointCloud<pcl::PointXYZ>);
      for (const auto& body : body_points)
      {
        const Eigen::Vector3f native =
            forward * body.x() + left * body.y() + up * body.z();
        lidar_points->push_back(makePoint(native));
        camera_points->push_back(makePoint(Eigen::Vector3f(-body.y(), -body.z(), body.x())));
      }

      pcl::PointCloud<pcl::PointXYZ>::Ptr sorted_camera(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr sorted_lidar(new pcl::PointCloud<pcl::PointXYZ>);
      ASSERT_TRUE(sortPatternCenters(camera_points, sorted_camera, "camera"));
      ASSERT_TRUE(sortPatternCenters(lidar_points, sorted_lidar, "lidar",
                                     forward_name, up_name));

      ASSERT_EQ(sorted_camera->size(), sorted_lidar->size());
      for (std::size_t i = 0; i < sorted_lidar->size(); ++i)
      {
        const auto& p = sorted_lidar->points[i];
        const Eigen::Vector3f optical =
            toOptical(Eigen::Vector3f(p.x, p.y, p.z), forward, left, up);
        EXPECT_NEAR(optical.x(), sorted_camera->points[i].x, 1e-5f);
        EXPECT_NEAR(optical.y(), sorted_camera->points[i].y, 1e-5f);
        EXPECT_NEAR(optical.z(), sorted_camera->points[i].z, 1e-5f);
      }
    }
  }

  EXPECT_EQ(valid_combinations, 24);
}

TEST(LidarMountAxes, RejectsInvalidOrParallelAxes)
{
  std::string error;
  EXPECT_FALSE(validateLidarMountAxes("+x", "-x", error));
  EXPECT_FALSE(validateLidarMountAxes("front", "+z", error));
  EXPECT_FALSE(validateLidarMountAxes("+x", "", error));
}

TEST(CameraCalibration, RequiresMatchingResolution)
{
  Params params{};
  params.camera_width = 2448;
  params.camera_height = 2048;
  params.fx = 2364.0;
  params.fy = 2368.0;
  params.cx = 1211.0;
  params.cy = 1040.0;
  params.k1 = -0.05;
  params.k2 = 0.12;
  params.p1 = 0.0;
  params.p2 = 0.0;

  std::string error;
  EXPECT_TRUE(validateCameraCalibrationForImage(params, 2448, 2048, error));
  EXPECT_FALSE(validateCameraCalibrationForImage(params, 1224, 1024, error));

  params.camera_width = 0;
  EXPECT_FALSE(validateCameraCalibrationForImage(params, 2448, 2048, error));
}

TEST(OutputDirectory, CreatesMissingParents)
{
  const std::string root =
      "/tmp/fast_calib_common_lib_test_" + std::to_string(static_cast<long long>(::getpid()));
  const std::string first = root + "/first";
  const std::string nested = first + "/second";

  std::string error;
  ASSERT_TRUE(ensureDirectoryTree(nested, error)) << error;

  struct stat status;
  ASSERT_EQ(::stat(nested.c_str(), &status), 0);
  EXPECT_TRUE(S_ISDIR(status.st_mode));

  EXPECT_EQ(::rmdir(nested.c_str()), 0);
  EXPECT_EQ(::rmdir(first.c_str()), 0);
  EXPECT_EQ(::rmdir(root.c_str()), 0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
