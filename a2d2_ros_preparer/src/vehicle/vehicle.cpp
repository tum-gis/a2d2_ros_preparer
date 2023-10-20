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

#include "vehicle.h"

#include <utility>
#include <glog/logging.h>
#include "../common/utility.h"
#include "bus_signal_stream.h"

namespace a2d2_ros_preparer {

    Vehicle::Vehicle(VehicleConfiguration vehicle_configuration,
                     BusSignalStream bus_signal_stream,
                     LidarScanStream lidar_scan_stream,
                     CameraImageStream camera_image_stream) :
                     vehicle_configuration_(std::move(vehicle_configuration)),
                     bus_signals_timeseries_(std::move(bus_signal_stream)),
                     lidar_scan_stream_(std::move(lidar_scan_stream)),
                     camera_image_stream_(std::move(camera_image_stream)) {}

    Time Vehicle::GetStartTimestamp() {
        return lidar_scan_stream_.GetStartTimestamp();
    }

    Time Vehicle::GetStopTimestamp() {
        return lidar_scan_stream_.GetStopTimestamp();
    }

    void Vehicle::WriteBusSignalsToGeoJSON(const std::filesystem::path& file_path, Duration time_delta, std::optional<Time> filter_start_timestamp, std::optional<Time> filter_stop_timestamp) {
        Time adjusted_start_timestamp = GetStartTimestamp();
        if (filter_start_timestamp.has_value())
            adjusted_start_timestamp = filter_start_timestamp.value();
        auto adjusted_stop_timestamp = GetStopTimestamp();
        if (filter_stop_timestamp.has_value())
            adjusted_stop_timestamp = filter_stop_timestamp.value();

        bus_signals_timeseries_.WriteToGeojsonFile(file_path, time_delta, adjusted_start_timestamp, adjusted_stop_timestamp);
    }

    void Vehicle::WriteTransformationDataToRosbag(rosbag::Bag& bag, std::optional<Time> filter_start_timestamp, std::optional<Time> filter_stop_timestamp, Duration time_delta) {
        CHECK_GT(time_delta, 0) << "Time delta duration must greater than zero.";

        auto valid_start_timestamp = GetValidStartTimestamp(filter_start_timestamp);
        auto valid_stop_timestamp = GetValidStopTimestamp(filter_stop_timestamp);

        Time current_timestamp = valid_start_timestamp;
        while (current_timestamp < valid_stop_timestamp) {

            tf2_msgs::TFMessage tf_messages;
            auto ros_timestamp = ToRosTime(current_timestamp);
            //tf_messages.transforms.push_back(vehicle_configuration_.GetOdomToBaseTransformMessage(ros_timestamp));
            tf_messages.transforms.push_back(vehicle_configuration_.GetImuToBaseTransformMessage(ros_timestamp));

            for (const auto& current_camera_identifier: vehicle_configuration_.GetCameraIdentifiers())
                tf_messages.transforms.push_back(vehicle_configuration_.GetCameraToBaseTransformMessage(current_camera_identifier, ros_timestamp));
            for (const auto& current_lidar_identifier: vehicle_configuration_.GetLidarIdentifiers())
                tf_messages.transforms.push_back(vehicle_configuration_.GetLidarToBaseTransformMessage(current_lidar_identifier, ros_timestamp));

            bag.write("/tf_static", ros_timestamp, tf_messages);

            current_timestamp += time_delta;
        }

    }

    void Vehicle::WriteImuDataToRosbag(rosbag::Bag& bag, std::optional<Time> filter_start_timestamp, std::optional<Time> filter_stop_timestamp, Duration time_delta) {
        CHECK_GT(time_delta, 0) << "Time delta duration must greater than zero.";

        LOG(INFO) << "Writing IMU data";
        auto valid_start_timestamp = GetValidStartTimestamp(filter_start_timestamp);
        auto valid_stop_timestamp = GetValidStopTimestamp(filter_stop_timestamp);

        Time current_timestamp = valid_start_timestamp;
        while (current_timestamp < valid_stop_timestamp) {
            auto current_imu_message = bus_signals_timeseries_.GetImuMessage(current_timestamp);
            bag.write("/imu", current_imu_message.header.stamp, current_imu_message);

            current_timestamp += time_delta;
        }
    }

    void Vehicle::WriteNavSatDataToRosbag(rosbag::Bag& bag, std::optional<Time> filter_start_timestamp, std::optional<Time> filter_stop_timestamp, Duration time_delta) {
        CHECK_GT(time_delta, 0) << "Time delta duration must greater than zero.";

        LOG(INFO) << "Writing NavSat data";
        auto valid_start_timestamp = GetValidStartTimestamp(filter_start_timestamp);
        auto valid_stop_timestamp = GetValidStopTimestamp(filter_stop_timestamp);

        Time current_timestamp = valid_start_timestamp;
        while (current_timestamp < valid_stop_timestamp) {
            auto current_navsat_message = bus_signals_timeseries_.GetNavSatMessage(current_timestamp);
            bag.write("/fix", current_navsat_message.header.stamp, current_navsat_message);

            current_timestamp += time_delta;
        }
    }

    void Vehicle::WriteOdometryDataToRosbag(rosbag::Bag& bag, std::optional<Time> filter_start_timestamp, std::optional<Time> filter_stop_timestamp, Duration time_delta) {
        CHECK_GT(time_delta, 0) << "Time delta duration must greater than zero.";
        LOG(INFO) << "Writing odometry data";
        auto valid_start_timestamp = GetValidStartTimestamp(filter_start_timestamp);
        auto valid_stop_timestamp = GetValidStopTimestamp(filter_stop_timestamp);

        auto odometry_messages = bus_signals_timeseries_.GetOdometryMessage(valid_start_timestamp, valid_stop_timestamp, time_delta);
        for (auto const& current_message: odometry_messages) {
            bag.write("/odom", current_message.header.stamp, current_message);
        }
    }

    void Vehicle::WriteLidarDataToXYZ(const std::filesystem::path &directory_path, std::optional<Time> filter_start_timestamp, std::optional<Time> filter_stop_timestamp, Duration time_delta) {
        CHECK_GT(time_delta, 0) << "Time delta duration must greater than zero.";
        LOG(INFO) << "Writing lidar data with a time_delta=" << time_delta << " to XYZ files to " << directory_path;
        auto valid_start_timestamp = GetValidStartTimestamp(filter_start_timestamp);
        auto valid_stop_timestamp = GetValidStopTimestamp(filter_stop_timestamp);

        std::filesystem::create_directory(directory_path);

        Time current_timestamp = valid_start_timestamp;

        unsigned int count = 1;
        unsigned int complete = (valid_stop_timestamp-valid_start_timestamp)/ time_delta;
        int complete_digits = std::to_string(complete).length();

        while (current_timestamp < valid_stop_timestamp) {
            auto current_timestamp_stop = current_timestamp + time_delta - 1;
            float current_percent = static_cast<float>(count)/static_cast<float>(complete) * 100;
            LOG(INFO) << "Writing lidar step " << count << "/" << complete << " (" << std::setprecision(2) << current_percent << "%)";

            auto filename = std::string(complete_digits - std::to_string(count).length(), '0') + std::to_string(count);

            lidar_scan_stream_.WriteAllDataToXYZFile(directory_path, filename, current_timestamp, current_timestamp_stop);
            current_timestamp += time_delta;
            count++;
        }
    }

    void Vehicle::WriteLidarDataToRosbag(rosbag::Bag& bag, std::optional<Time> filter_start_timestamp, std::optional<Time> filter_stop_timestamp, Duration time_delta) {
        CHECK_GT(time_delta, 0) << "Time delta duration must greater than zero.";
        auto valid_start_timestamp = GetValidStartTimestamp(filter_start_timestamp);
        auto valid_stop_timestamp = GetValidStopTimestamp(filter_stop_timestamp);

        Time current_timestamp = valid_start_timestamp;

        unsigned int count = 1;
        unsigned int complete = (valid_stop_timestamp-valid_start_timestamp)/ time_delta;

        while (current_timestamp < valid_stop_timestamp) {
            auto current_timestamp_stop = current_timestamp + time_delta - 1;
            float current_percent = static_cast<float>(count)/static_cast<float>(complete) * 100;
            LOG(INFO) << "Writing lidar step " << count << "/" << complete << " (" << std::setprecision(3) << current_percent << "%)";

            lidar_scan_stream_.WriteAllDataToRosbag(bag, current_timestamp, current_timestamp_stop);
            current_timestamp += time_delta;
            count++;
        }

    }

    void Vehicle::WriteDistortedCameraDataToRosbag(rosbag::Bag &bag, std::optional<Time> filter_start_timestamp, std::optional<Time> filter_stop_timestamp) {
        auto valid_start_timestamp = GetValidStartTimestamp(filter_start_timestamp);
        auto valid_stop_timestamp = GetValidStopTimestamp(filter_stop_timestamp);

        for (const auto& current_camera_identifier: camera_image_stream_.GetCameraIdentifiers()) {
            LOG(INFO) << "Writing distorted images for camera '" << current_camera_identifier << "'";
            auto sequence_ids = lidar_scan_stream_.GetAllDataSequenceIds(current_camera_identifier, valid_start_timestamp, valid_stop_timestamp);
            camera_image_stream_.WriteDistortedCameraDataToRosbag(bag, current_camera_identifier, sequence_ids);
        }
    }

    void Vehicle::WriteRectifiedCameraDataToRosbag(rosbag::Bag &bag, std::optional<Time> filter_start_timestamp, std::optional<Time> filter_stop_timestamp) {
        auto valid_start_timestamp = GetValidStartTimestamp(filter_start_timestamp);
        auto valid_stop_timestamp = GetValidStopTimestamp(filter_stop_timestamp);

        for (const auto& current_camera_identifier: camera_image_stream_.GetCameraIdentifiers()) {
            LOG(INFO) << "Writing rectified images for camera '" << current_camera_identifier << "'";
            auto sequence_ids = lidar_scan_stream_.GetAllDataSequenceIds(current_camera_identifier, valid_start_timestamp, valid_stop_timestamp);
            camera_image_stream_.WriteRectifiedCameraDataToRosbag(bag, current_camera_identifier, sequence_ids);
        }
    }

    void Vehicle::WriteCameraInfoDataToRosbag(rosbag::Bag &bag, std::optional<Time> filter_start_timestamp, std::optional<Time> filter_stop_timestamp) {
        auto valid_start_timestamp = GetValidStartTimestamp(filter_start_timestamp);
        auto valid_stop_timestamp = GetValidStopTimestamp(filter_stop_timestamp);

        for (const auto& current_camera_identifier: camera_image_stream_.GetCameraIdentifiers()) {
            LOG(INFO) << "Writing camera info data for camera '" << current_camera_identifier << "'";
            auto sequence_ids = lidar_scan_stream_.GetAllDataSequenceIds(current_camera_identifier, valid_start_timestamp, valid_stop_timestamp);
            camera_image_stream_.WriteCameraInfoDataToRosbag(bag, current_camera_identifier, sequence_ids);
        }
    }

    Time Vehicle::GetValidStartTimestamp(std::optional<Time> filter_start_timestamp) {
        if (filter_start_timestamp.has_value()) {
            CHECK_GE(filter_start_timestamp.value(), GetStartTimestamp())
                << std::setprecision(std::numeric_limits<double>::digits10 + 2)
                << "Filter start time (" << ToUniversalSeconds(filter_start_timestamp.value())
                << "s) must be greater equal than the start timestamp ("
                << ToUniversalSeconds(GetStartTimestamp()) << "s) of the vehicle data";
            CHECK_LE(filter_start_timestamp.value(), GetStopTimestamp())
                << std::setprecision(std::numeric_limits<double>::digits10 + 2)
                << "Filter start time (" << ToUniversalSeconds(filter_start_timestamp.value())
                << "s) must be lower equal than the stop timestamp ("
                << ToUniversalSeconds(GetStopTimestamp()) << "s) of the vehicle data";
            return filter_start_timestamp.value();
        } else
            return GetStartTimestamp();
    }

    Time Vehicle::GetValidStopTimestamp(std::optional<Time> filter_stop_timestamp) {
        if (filter_stop_timestamp.has_value()) {
            CHECK_GE(filter_stop_timestamp.value(), GetStartTimestamp())
                << std::setprecision(std::numeric_limits<double>::digits10 + 2)
                << "Filter stop timestamp (" << ToUniversalSeconds(filter_stop_timestamp.value())
                << "s) must be greater equal than the start timestamp (" <<
                ToUniversalSeconds(GetStartTimestamp()) << "s) of the vehicle data";
            CHECK_LE(filter_stop_timestamp.value(), GetStopTimestamp())
                << std::setprecision(std::numeric_limits<double>::digits10 + 2) <<
                "Filter stop timestamp (" << ToUniversalSeconds(filter_stop_timestamp.value())
                << "s) must be lower equal than the stop timestamp ("
                << ToUniversalSeconds(GetStopTimestamp()) << "s) of the vehicle data";
            return filter_stop_timestamp.value();
        } else
            return GetStopTimestamp();
    }

}
