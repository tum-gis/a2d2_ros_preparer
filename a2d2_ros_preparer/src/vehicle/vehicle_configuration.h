/*
 * Copyright 2021-2021 Chair of Geoinformatics, Technical University of Munich
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef A2D2_ROS_PREPARER_VEHICLE_CONFIGURATION_H
#define A2D2_ROS_PREPARER_VEHICLE_CONFIGURATION_H

#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/CameraInfo.h>

#include "../common/types.h"
#include "../common/timing.h"
#include "../camera/camera_configuration.h"

namespace a2d2_ros_preparer {

    class VehicleConfiguration {
    public:
        explicit VehicleConfiguration(const std::filesystem::path& data_directory_path,
                                      const std::filesystem::path& sensor_configuration_filepath);

        std::vector<CameraDirectionIdentifier> GetCameraIdentifiers();
        std::vector<LidarDirectionIdentifier> GetLidarIdentifiers();

        geometry_msgs::TransformStamped GetOdomToBaseTransformMessage(ros::Time timestamp);
        geometry_msgs::TransformStamped GetImuToBaseTransformMessage(ros::Time timestamp);

        geometry_msgs::TransformStamped GetCameraToBaseTransformMessage(const CameraDirectionIdentifier& camera_identifier, ros::Time timestamp);
        Eigen::Affine3d GetCameraToBaseAffineTransform(const CameraDirectionIdentifier& camera_identifier);
        std::map<CameraDirectionIdentifier, Eigen::Affine3d> GetCameraToBaseAffineTransforms();

        CameraConfiguration GetCameraConfiguration(const CameraDirectionIdentifier& camera_identifier);
        std::map<CameraDirectionIdentifier, CameraConfiguration> GetCameraConfigurations();

        geometry_msgs::TransformStamped GetLidarToBaseTransformMessage(const LidarDirectionIdentifier& lidar_identifier, ros::Time timestamp);
        Eigen::Affine3d GetLidarToBaseAffineTransform(const LidarDirectionIdentifier& lidar_identifier);
        std::map<LidarDirectionIdentifier, Eigen::Affine3d> GetLidarToBaseAffineTransforms();

        [[nodiscard]] std::filesystem::path GetLidarDataDirectoryPath() const { return data_directory_path_ / "lidar"; };
        [[nodiscard]] std::filesystem::path GetCameraDataDirectoryPath() const { return data_directory_path_ / "camera"; };

    private:
        std::filesystem::path data_directory_path_;

        nlohmann::json sensor_configuration_json_;
    };

}
#endif //A2D2_ROS_PREPARER_VEHICLE_CONFIGURATION_H
