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


#include "options_parser.h"

#include <iostream>
#include <glog/logging.h>

namespace a2d2_ros_preparer {

    Options OptionsParser::Parse() {
        ros::NodeHandle node;
        std::string prefix = "/converter/options";

        auto options = Options();

        ParsePaths(node, prefix + "/paths", options);
        ParseFilter(node, prefix + "/filter", options);
        ParseCameraSensors(node, prefix + "/camera_sensors", options);
        ParseBusSignals(node, prefix + "/bus_signals", options);
        ParsePublish(node, prefix + "/publish", options);
        ParseWrites(node, prefix + "/write", options);

        return options;
    }

    void OptionsParser::ParsePaths(const ros::NodeHandle& node, const std::string& prefix, Options& options) {
        if (node.hasParam(prefix + "/source_sensor_data_directory"))
            options.SetSourceSensorDataDirectory(GetParam<std::string>(node, prefix + "/source_sensor_data_directory"));
        if (node.hasParam(prefix + "/source_bus_signals_filepath"))
            options.SetSourceBusSignalsFilepath(GetParam<std::string>(node, prefix + "/source_bus_signals_filepath"));
        if (node.hasParam(prefix + "/source_sensor_configuration_filepath"))
            options.SetSourceSensorConfigurationFilepath(GetParam<std::string>(node, prefix + "/source_sensor_configuration_filepath"));

        if (node.hasParam(prefix + "/target_directory"))
            options.SetTargetDirectory(GetParam<std::string>(node, prefix + "/target_directory"));

    }

    void OptionsParser::ParseFilter(const ros::NodeHandle& node, const std::string& prefix, Options& options) {
        if (node.hasParam(prefix + "/start_timestamp"))
            options.SetFilterStartTimestamp(GetParam<double>(node, prefix + "/start_timestamp"));
        if (node.hasParam(prefix + "/stop_timestamp"))
            options.SetFilterStopTimestamp(GetParam<double>(node, prefix + "/stop_timestamp"));
        if (node.hasParam(prefix + "/camera_identifiers"))
            options.SetFilterCameraIdentifiers(GetParam<std::vector<std::string>>(node, prefix + "/camera_identifiers"));
    }

    void OptionsParser::ParseCameraSensors(const ros::NodeHandle& node, const std::string& prefix, Options& options) {
        if (node.hasParam(prefix + "/fieldnames/image_timestamp"))
            options.SetFieldNameImageTime(GetParam<std::string>(node, prefix + "/fieldnames/image_timestamp"));

        if (node.hasParam(prefix + "/time_estimation/lidar_timeframe_to_image_ratio"))
            options.SetCameraSensorsTimeEstimationLidarTimeframeToImageRatio(GetParam<double>(node, prefix + "/time_estimation/lidar_timeframe_to_image_ratio"));
        if (node.hasParam(prefix + "/time_estimation/validate_method"))
            options.SetCameraSensorsTimeEstimationValidateMethod(GetParam<bool>(node, prefix + "/time_estimation/validate_method"));
        if (node.hasParam(prefix + "/time_estimation/time_tolerance_sec"))
            options.SetCameraSensorsTimeEstimationTimeTolerance(GetParam<double>(node, prefix + "/time_estimation/time_tolerance_sec"));
    }

    void OptionsParser::ParseBusSignals(const ros::NodeHandle& node, std::string prefix, Options& options) {
        if (node.hasParam(prefix + "/time_offset_tolerance"))
            options.SetTimeOffsetTolerance(GetParam<double>(node, prefix + "/time_offset_tolerance"));
        if (node.hasParam(prefix + "/altitude_offset"))
            options.SetAltitudeOffset(GetParam<double>(node, prefix + "/altitude_offset"));

        prefix = prefix + "/fieldnames";
        if (node.hasParam(prefix + "/velocity"))
            options.SetFieldNameVelocity(GetParam<std::string>(node, prefix + "/velocity"));

        if (node.hasParam(prefix + "/acceleration_x"))
            options.SetFieldNameAccelerationX(GetParam<std::string>(node, prefix + "/acceleration_x"));
        if (node.hasParam(prefix + "/acceleration_y"))
            options.SetFieldNameAccelerationY(GetParam<std::string>(node, prefix + "/acceleration_y"));
        if (node.hasParam(prefix + "/acceleration_z"))
            options.SetFieldNameAccelerationZ(GetParam<std::string>(node, prefix + "/acceleration_z"));

        if (node.hasParam(prefix + "/angular_velocity_x"))
            options.SetFieldNameAngularVelocityX(GetParam<std::string>(node, prefix + "/angular_velocity_x"));
        if (node.hasParam(prefix + "/angular_velocity_y"))
            options.SetFieldNameAngularVelocityY(GetParam<std::string>(node, prefix + "/angular_velocity_y"));
        if (node.hasParam(prefix + "/angular_velocity_z"))
            options.SetFieldNameAngularVelocityZ(GetParam<std::string>(node, prefix + "/angular_velocity_z"));

        if (node.hasParam(prefix + "/latitude_degree"))
            options.SetFieldNameLatitudeDegree(GetParam<std::string>(node, prefix + "/latitude_degree"));
        if (node.hasParam(prefix + "/longitude_degree"))
            options.SetFieldNameLongitudeDegree(GetParam<std::string>(node, prefix + "/longitude_degree"));
        if (node.hasParam(prefix + "/altitude"))
            options.SetFieldNameAltitude(GetParam<std::string>(node, prefix + "/altitude"));
    }

    void OptionsParser::ParsePublish(const ros::NodeHandle& node, const std::string& prefix, Options& options) {
        if (node.hasParam(prefix + "/trajectory_publish_period_sec")) {
            auto value = boost::lexical_cast<double>(GetParam<std::string>(node, prefix + "/trajectory_publish_period_sec"));
            options.SetTrajectoryPublishPeriod(value);
        }
        if (node.hasParam(prefix + "/tf_publish_period_sec")) {
            auto value = boost::lexical_cast<double>(GetParam<std::string>(node, prefix + "/tf_publish_period_sec"));
            options.SetTfPublishPeriod(value);
        }
        if (node.hasParam(prefix + "/imu_publish_period_sec")) {
            auto value = boost::lexical_cast<double>(GetParam<std::string>(node, prefix + "/imu_publish_period_sec"));
            options.SetImuPublishPeriod(value);
        }
        if (node.hasParam(prefix + "/lidar_publish_period_sec")) {
            auto value = boost::lexical_cast<double>(GetParam<std::string>(node, prefix + "/lidar_publish_period_sec"));
            options.SetLidarPublishPeriod(value);
        }
    }

    void OptionsParser::ParseWrites(const ros::NodeHandle& node, const std::string& prefix, Options& options) {
        if (node.hasParam(prefix + "/lidar_data_xyz"))
            options.SetWriteLidarDataXYZ(GetParam<bool>(node, prefix + "/lidar_data_xyz"));
        if (node.hasParam(prefix + "/lidar_data_rosbag"))
            options.SetWriteLidarDataRosbag(GetParam<bool>(node, prefix + "/lidar_data_rosbag"));
        if (node.hasParam(prefix + "/camera_data_rosbag"))
            options.SetWriteCameraDataRosbag(GetParam<bool>(node, prefix + "/camera_data_rosbag"));
    }
}
