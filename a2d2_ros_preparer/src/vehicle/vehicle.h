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


#ifndef A2D2_ROS_PREPARER_VEHICLE_H
#define A2D2_ROS_PREPARER_VEHICLE_H

#include <ros/node_handle.h>
#include <rosbag/bag.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "../common/timing.h"
#include "../common/types.h"
#include "../lidar/lidar_scan_timeseries_per_view.h"
#include "../lidar/lidar_scan_stream.h"
#include "../camera/camera_image_stream.h"
#include "vehicle_configuration.h"
#include "bus_signal_stream.h"

namespace a2d2_ros_preparer {

    class Vehicle {

    public:
        explicit Vehicle(VehicleConfiguration vehicle_configuration,
                         BusSignalStream bus_signal_stream,
                         LidarScanStream lidar_scan_stream,
                         CameraImageStream camera_image_stream);

        Time GetStartTimestamp();
        Time GetStopTimestamp();

        void WriteBusSignalsToGeoJSON(const std::filesystem::path& file_path,
                                      Duration time_delta,
                                      std::optional<Time> filter_start_timestamp = {},
                                      std::optional<Time> filter_stop_timestamp = {});

        void WriteLidarDataToXYZ(const std::filesystem::path& directory_path, std::optional<Time> filter_start_timestamp, std::optional<Time> filter_stop_timestamp, Duration time_delta);

        void WriteTransformationDataToRosbag(rosbag::Bag& bag, std::optional<Time> filter_start_timestamp, std::optional<Time> filter_stop_timestamp, Duration time_delta);
        void WriteImuDataToRosbag(rosbag::Bag& bag, std::optional<Time> filter_start_timestamp, std::optional<Time> filter_stop_timestamp, Duration time_delta);
        void WriteNavSatDataToRosbag(rosbag::Bag& bag, std::optional<Time> filter_start_timestamp, std::optional<Time> filter_stop_timestamp, Duration time_delta);
        void WriteOdometryDataToRosbag(rosbag::Bag& bag, std::optional<Time> filter_start_timestamp, std::optional<Time> filter_stop_timestamp, Duration time_delta);
        void WriteLidarDataToRosbag(rosbag::Bag& bag, std::optional<Time> filter_start_timestamp, std::optional<Time> filter_stop_timestamp, Duration time_delta);

        void WriteDistortedCameraDataToRosbag(rosbag::Bag& bag, std::optional<Time> filter_start_timestamp, std::optional<Time> filter_stop_timestamp);
        void WriteRectifiedCameraDataToRosbag(rosbag::Bag& bag, std::optional<Time> filter_start_timestamp, std::optional<Time> filter_stop_timestamp);
        void WriteCameraInfoDataToRosbag(rosbag::Bag& bag, std::optional<Time> filter_start_timestamp, std::optional<Time> filter_stop_timestamp);

    private:
        VehicleConfiguration vehicle_configuration_;
        BusSignalStream bus_signals_timeseries_;
        LidarScanStream lidar_scan_stream_;
        CameraImageStream camera_image_stream_;

        Time GetValidStartTimestamp(std::optional<Time> filter_start_timestamp);
        Time GetValidStopTimestamp(std::optional<Time> filter_stop_timestamp);
    };
}

#endif //A2D2_ROS_PREPARER_VEHICLE_H
