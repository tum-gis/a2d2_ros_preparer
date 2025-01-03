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


#include "converter.h"

#include <sstream>
#include <rosbag/bag.h>
#include <glog/logging.h>
#include "vehicle/vehicle_configuration.h"
#include "common/timing.h"

namespace a2d2_ros_preparer {

    void Converter::Process(const Options& options) {
        LOG(INFO) << "Starting converter with source directory at " << options.source_sensor_data_directory();

        auto vehicle = Read(options);

        vehicle.WriteBusSignalsToGeoJSON(options.target_trajectories_geojson_filtered_filepath(), options.trajectory_publish_period(), options.filter_start_timestamp(), options.filter_stop_timestamp());

        if (options.write_lidar_data_xyz())
            vehicle.WriteLidarDataToXYZ(options.target_sensor_lidar_directory(), options.filter_start_timestamp(), options.filter_stop_timestamp(), options.lidar_publish_period());

        WriteRosbag(options, vehicle);
    }

    Vehicle Converter::Read(const Options &options) {

        auto vehicle_configuration = VehicleConfiguration(options.source_sensor_data_directory(),
                                                          options.source_sensor_configuration_filepath());
        auto lidar_scan_stream = LidarScanStream(vehicle_configuration.GetLidarDataDirectoryPath(),
                                                 vehicle_configuration.GetCameraIdentifiers(),
                                                 vehicle_configuration.GetLidarIdentifiers(),
                                                 options.lidar_sensor_id_remappings(),
                                                 vehicle_configuration.GetCameraToBaseAffineTransforms(),
                                                 vehicle_configuration.GetLidarToBaseAffineTransforms(),
                                                 options.lidar_correction_transformation(),
                                                 options.filter_start_timestamp(),
                                                 options.filter_stop_timestamp());

        auto time_rectime_offset = lidar_scan_stream.GetTimeToRectimeOffsetMedian(options.time_offset_tolerance());
        LOG(INFO) << "Identified time to rectime offset of " << ToSeconds(time_rectime_offset) << "s";

        auto bus_signal_stream = BusSignalStream(options, time_rectime_offset);
        bus_signal_stream.WriteToGeojsonFile(options.target_trajectories_geojson_complete_filepath(), options.trajectory_publish_period());


        auto camera_image_stream = CameraImageStream(vehicle_configuration.GetCameraDataDirectoryPath(),
                                                     vehicle_configuration.GetCameraIdentifiers(),
                                                     options.filter_camera_identifiers(),
                                                     vehicle_configuration.GetCameraConfigurations(),
                                                     lidar_scan_stream.GetPointCloudTimeMinsPerCameraDirection(),
                                                     lidar_scan_stream.GetPointCloudTimeMaxsPerCameraDirection(),
                                                     time_rectime_offset,
                                                     options.field_name_image_time(),
                                                     options.camera_sensors_time_estimation_lidar_timeframe_to_image_ratio(),
                                                     options.camera_sensors_time_estimation_time_tolerance());

        auto vehicle = Vehicle(vehicle_configuration,
                               bus_signal_stream,
                               lidar_scan_stream,
                               camera_image_stream);
        return vehicle;
    }

    void Converter::WriteRosbag(const Options& options, Vehicle vehicle) {
        LOG(INFO) << "Writing rosbag file to " << options.target_rosbag_filepath();
        rosbag::Bag bag;
        bag.open(options.target_rosbag_filepath(), rosbag::bagmode::Write);
        vehicle.WriteTransformationDataToRosbag(bag, options.filter_start_timestamp(), options.filter_stop_timestamp(),
                                                options.tf_publish_period());

        vehicle.WriteImuDataToRosbag(bag, options.filter_start_timestamp(), options.filter_stop_timestamp(), options.imu_publish_period());
        vehicle.WriteNavSatDataToRosbag(bag, options.filter_start_timestamp(), options.filter_stop_timestamp(), options.imu_publish_period());
        vehicle.WriteOdometryDataToRosbag(bag, options.filter_start_timestamp(), options.filter_stop_timestamp(), options.imu_publish_period());

        if (options.write_lidar_data_rosbag())
            vehicle.WriteLidarDataToRosbag(bag, options.filter_start_timestamp(), options.filter_stop_timestamp(), options.lidar_publish_period());
        if (options.write_distorted_camera_data_rosbag())
            vehicle.WriteDistortedCameraDataToRosbag(bag, options.filter_start_timestamp(), options.filter_stop_timestamp());
        if (options.write_rectified_camera_data_rosbag())
            vehicle.WriteRectifiedCameraDataToRosbag(bag, options.filter_start_timestamp(), options.filter_stop_timestamp());
        if (options.write_distorted_camera_data_rosbag() || options.write_rectified_camera_data_rosbag())
            vehicle.WriteCameraInfoDataToRosbag(bag, options.filter_start_timestamp(), options.filter_stop_timestamp());

        bag.close();
    }

}
