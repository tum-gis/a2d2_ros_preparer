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


#include "vehicle_configuration.h"

#include <math.h>
#include <iostream>
#include <eigen_conversions/eigen_msg.h>
#include <tf2_msgs/TFMessage.h>
#include <glog/logging.h>
#include "../common/vector_conversion.h"

namespace a2d2_ros_preparer {

    VehicleConfiguration::VehicleConfiguration(const std::filesystem::path& data_directory_path,
                                               const std::filesystem::path& sensor_configuration_filepath) {

        CHECK(exists(data_directory_path)) << "No directory found at " << data_directory_path;
        CHECK(exists(sensor_configuration_filepath)) << "No sensor configuration file found at " << sensor_configuration_filepath;

        data_directory_path_ = data_directory_path;

        std::ifstream jsonFile(sensor_configuration_filepath);
        jsonFile >> sensor_configuration_json_;
    }

    std::vector<CameraDirectionIdentifier> VehicleConfiguration::GetCameraIdentifiers() {
        std::vector<std::string> camera_identifiers;
        auto cameras_config_node = sensor_configuration_json_["cameras"];

        for (auto it = cameras_config_node.begin(); it != cameras_config_node.end(); ++it)
            camera_identifiers.push_back(it.key());

        return camera_identifiers;
    }

    std::vector<LidarDirectionIdentifier> VehicleConfiguration::GetLidarIdentifiers() {
        std::vector<std::string> lidar_identifiers;
        auto lidars_config_node = sensor_configuration_json_["lidars"];

        for (auto it = lidars_config_node.begin(); it != lidars_config_node.end(); ++it)
            lidar_identifiers.push_back(it.key());

        return lidar_identifiers;
    }

    Eigen::Affine3d VehicleConfiguration::GetCameraToBaseAffineTransform(const CameraDirectionIdentifier& camera_identifier) {
        nlohmann::json vehicle_view = sensor_configuration_json_["vehicle"]["view"];
        nlohmann::json camera_view = sensor_configuration_json_["cameras"][camera_identifier]["view"];
        return GetTransformFromTo(camera_view, vehicle_view, 1.0e-10);
    }

    Eigen::Affine3d VehicleConfiguration::GetLidarToBaseAffineTransform(const LidarDirectionIdentifier& lidar_identifier) {
        nlohmann::json vehicle_view = sensor_configuration_json_["vehicle"]["view"];
        nlohmann::json lidar_view = sensor_configuration_json_["lidars"][lidar_identifier]["view"];
        return GetTransformFromTo(lidar_view, vehicle_view, 1.0e-10);
    }

    geometry_msgs::TransformStamped VehicleConfiguration::GetCameraToBaseTransformMessage(const CameraDirectionIdentifier& camera_identifier, ros::Time timestamp) {

        // geometry
        auto affine = GetCameraToBaseAffineTransform(camera_identifier);

        // rotation adjustment for camera orientation according to ros
        // see: https://wiki.ros.org/image_pipeline/CameraInfo
        Eigen::AngleAxisd rollAngle(-M_PI_2, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(M_PI_2, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(0, Eigen::Vector3d::UnitZ());
        Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
        Eigen::Matrix3d rotationMatrix = q.matrix();
        Eigen::Affine3d rotationAdjustmentAffine(rotationMatrix);


        geometry_msgs::Transform geometry_message;
        tf::transformEigenToMsg(affine * rotationAdjustmentAffine, geometry_message);

        // transform
        geometry_msgs::TransformStamped transform_message;
        transform_message.transform = geometry_message;
        transform_message.header.stamp = timestamp;
        transform_message.header.frame_id = "base_link";
        transform_message.child_frame_id = "camera_" + camera_identifier;

        return transform_message;
    }

    geometry_msgs::TransformStamped VehicleConfiguration::GetLidarToBaseTransformMessage(const LidarDirectionIdentifier& lidar_identifier, ros::Time timestamp) {

        geometry_msgs::Transform geometry_message;
        tf::transformEigenToMsg(GetLidarToBaseAffineTransform(lidar_identifier), geometry_message);

        // transform
        geometry_msgs::TransformStamped transform_message;
        transform_message.transform = geometry_message;
        transform_message.header.stamp = timestamp;
        transform_message.header.frame_id = "base_link";
        transform_message.child_frame_id = "lidar_" + lidar_identifier;

        return transform_message;
    }

    geometry_msgs::TransformStamped VehicleConfiguration::GetOdomToBaseTransformMessage(ros::Time timestamp) {

        // geometry
        Eigen::Affine3d affine = Eigen::Affine3d::Identity();
        geometry_msgs::Transform geometry_message;
        tf::transformEigenToMsg(affine, geometry_message);

        // transform message
        geometry_msgs::TransformStamped message;
        message.transform = geometry_message;
        message.header.stamp = timestamp;
        message.header.frame_id = "base_link";
        message.child_frame_id = "odom";

        return message;
    }

    geometry_msgs::TransformStamped VehicleConfiguration::GetImuToBaseTransformMessage(ros::Time timestamp) {

        // geometry
        /*Eigen::AngleAxisd rollAngle(0, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(0, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(0, Eigen::Vector3d::UnitZ());
        Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
        Eigen::Matrix3d rotationMatrix = q.matrix();
        Eigen::Affine3d affine(rotationMatrix);*/
        Eigen::Affine3d affine = Eigen::Affine3d::Identity();
        geometry_msgs::Transform geometry_message;
        tf::transformEigenToMsg(affine, geometry_message);

        // transform
        geometry_msgs::TransformStamped transform_message;
        transform_message.transform = geometry_message;
        transform_message.header.stamp = timestamp;
        transform_message.header.frame_id = "base_link";
        transform_message.child_frame_id = "imu_link";

        return transform_message;
    }

    std::map<LidarDirectionIdentifier, Eigen::Affine3d> VehicleConfiguration::GetLidarToBaseAffineTransforms() {
        std::map<LidarDirectionIdentifier, Eigen::Affine3d> lidar_affine_transforms;
        for (const auto& current_lidar_id : GetLidarIdentifiers()) {
            lidar_affine_transforms.insert(std::make_pair(current_lidar_id, GetLidarToBaseAffineTransform(current_lidar_id)));
        }

        return lidar_affine_transforms;
    }

    std::map<CameraDirectionIdentifier, Eigen::Affine3d> VehicleConfiguration::GetCameraToBaseAffineTransforms() {
        std::map<CameraDirectionIdentifier, Eigen::Affine3d> camera_affine_transforms;
        for (const auto& current_lidar_id : GetCameraIdentifiers()) {
            camera_affine_transforms.insert(std::make_pair(current_lidar_id, GetCameraToBaseAffineTransform(current_lidar_id)));
        }

        return camera_affine_transforms;
    }

    CameraConfiguration VehicleConfiguration::GetCameraConfiguration(const CameraDirectionIdentifier& camera_identifier) {
        auto camera_config_json = sensor_configuration_json_["cameras"][camera_identifier];

        uint32_t width = camera_config_json["Resolution"][0];
        uint32_t height = camera_config_json["Resolution"][1];

        CameraLensType camera_lens_type = CameraLensType::TELECAM;
        std::string lens_type_string = camera_config_json["Lens"];
        if (lens_type_string == "Telecam")
            camera_lens_type = CameraLensType::TELECAM;
        else if (lens_type_string == "Fisheye")
            camera_lens_type = CameraLensType::FISHEYE;
        else
            LOG(WARNING) << "Unknown lens type in configuration (" << lens_type_string << "). Defaulting to telecam.";


        Eigen::Matrix3d camera_matrix = ToEigenMatrix3d(camera_config_json["CamMatrix"]);
        Eigen::Matrix3d camera_matrix_original = ToEigenMatrix3d(camera_config_json["CamMatrixOriginal"]);
        auto distortion_std = camera_config_json["Distortion"][0].get<std::vector<double>>();

        return CameraConfiguration(width, height, camera_lens_type, camera_matrix, camera_matrix_original, distortion_std);
    }

    std::map<CameraDirectionIdentifier, CameraConfiguration> VehicleConfiguration::GetCameraConfigurations() {
        auto camera_configurations = std::map<CameraDirectionIdentifier, CameraConfiguration>();

        for (const auto& current_camera_identifier: GetCameraIdentifiers())
            camera_configurations.insert(std::make_pair(current_camera_identifier, GetCameraConfiguration(current_camera_identifier)));

        return camera_configurations;
    }
}
