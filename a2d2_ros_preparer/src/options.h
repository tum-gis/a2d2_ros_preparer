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


#ifndef A2D2_ROS_PREPARER_CONVERTER_OPTIONS_H
#define A2D2_ROS_PREPARER_CONVERTER_OPTIONS_H

#include <filesystem>
#include <cmath>
#include <utility>
#include <glog/logging.h>
#include "common/timing.h"
#include "common/types.h"

namespace a2d2_ros_preparer {

    class Options {
    public:
        explicit Options() = default;

        // paths
        void SetSourceSensorDataDirectory(const std::filesystem::path& path) { source_sensor_data_directory_ = path; };
        void SetSourceBusSignalsFilepath(const std::filesystem::path& path) { source_bus_signals_filepath_ = path; };
        void SetSourceSensorConfigurationFilepath(const std::filesystem::path& path) { source_sensor_configuration_filepath_ = path; };
        void SetTargetDirectory(const std::filesystem::path& path) { target_directory_ = path; };
        void SetTargetTrajectoriesSubdirectory(const std::string& name) { target_trajectories_subdirectory_name_ = name; }

        [[nodiscard]] std::filesystem::path source_sensor_data_directory() const { return source_sensor_data_directory_; }
        [[nodiscard]] std::filesystem::path source_bus_signals_filepath() const;
        [[nodiscard]] std::filesystem::path source_sensor_configuration_filepath() const;

        [[nodiscard]] std::filesystem::path target_directory() const { return target_directory_; }
        [[nodiscard]] std::filesystem::path target_trajectories_directory() const { return target_directory_ / target_trajectories_subdirectory_name_; }
        [[nodiscard]] std::filesystem::path target_trajectories_geojson_complete_filepath() const { return target_trajectories_directory() / "bus_signals_trajectory_complete.geojson"; }
        [[nodiscard]] std::filesystem::path target_trajectories_geojson_filtered_filepath() const { return target_trajectories_directory() / "bus_signals_trajectory_filtered.geojson"; }
        [[nodiscard]] std::filesystem::path target_rosbag_filepath() const { return target_directory() / "driving_dataset.bag"; }
        [[nodiscard]] std::filesystem::path target_sensor_lidar_directory() const { return target_directory() / "driving_dataset_sensor_lidar"; }

        // filter
        void SetFilterStartTimestamp(double start_timestamp);
        void SetFilterStopTimestamp(double stop_timestamp);
        void SetFilterCameraIdentifiers(const std::vector<std::string>& camera_identifiers);

        [[nodiscard]] std::optional<Time> filter_start_timestamp() const { return filter_start_timestamp_; }
        [[nodiscard]] std::optional<Time> filter_stop_timestamp() const { return filter_stop_timestamp_; }
        [[nodiscard]] std::set<CameraDirectionIdentifier> filter_camera_identifiers() const { return filter_camera_identifiers_; }

        // lidar sensors
        void SetLidarSensorsIdRemappings(std::map<CameraDirectionIdentifier, std::map<uint64_t, uint64_t>> remappings) { lidar_sensor_id_remappings_ = std::move(remappings); }

        [[nodiscard]] std::map<CameraDirectionIdentifier, std::map<uint64_t, uint64_t>> lidar_sensor_id_remappings() const { return lidar_sensor_id_remappings_; }


        // camera sensors
        void SetFieldNameImageTime(std::string field_name) { field_name_image_time_ = std::move(field_name); }
        void SetCameraSensorsTimeEstimationLidarTimeframeToImageRatio(double ratio) { camera_sensors_time_estimation_lidar_timeframe_to_image_ratio_ = ratio; }
        void SetCameraSensorsTimeEstimationValidateMethod(bool validate_method) { camera_sensors_time_estimation_validate_method_ = validate_method; }
        void SetCameraSensorsTimeEstimationTimeTolerance(double time_tolerance_sec);

        [[nodiscard]] std::string field_name_image_time() const { return field_name_image_time_; }
        [[nodiscard]] double_t camera_sensors_time_estimation_lidar_timeframe_to_image_ratio() const { return camera_sensors_time_estimation_lidar_timeframe_to_image_ratio_; }
        [[nodiscard]] std::optional<Duration> camera_sensors_time_estimation_time_tolerance() const;

        // bus signals
        void SetTimeOffsetTolerance(double seconds);
        void SetAltitudeOffset(double offset);

        void SetFieldNameVelocity(std::string field_name) { field_name_velocity_ = std::move(field_name); }
        void SetFieldNameAccelerationX(std::string field_name) { field_name_acceleration_x_ = std::move(field_name); }
        void SetFieldNameAccelerationY(std::string field_name) { field_name_acceleration_y_ = std::move(field_name); }
        void SetFieldNameAccelerationZ(std::string field_name) { field_name_acceleration_z_ = std::move(field_name); }
        void SetFieldNameAngularVelocityX(std::string field_name) { field_name_angular_velocity_x_ = std::move(field_name); }
        void SetFieldNameAngularVelocityY(std::string field_name) { field_name_angular_velocity_y_ = std::move(field_name); }
        void SetFieldNameAngularVelocityZ(std::string field_name) { field_name_angular_velocity_z_ = std::move(field_name); }
        void SetFieldNameLatitudeDegree(std::string field_name) { field_name_latitude_degree_ = std::move(field_name); }
        void SetFieldNameLongitudeDegree(std::string field_name) { field_name_longitude_degree_ = std::move(field_name); }
        void SetFieldNameAltitude(std::string field_name) { field_name_altitude_ = std::move(field_name); }


        [[nodiscard]] Duration time_offset_tolerance() const { return time_offset_tolerance_; }
        [[nodiscard]] double_t altitude_offset() const { return altitude_offset_; }

        [[nodiscard]] std::string field_name_velocity() const { return field_name_velocity_; }
        [[nodiscard]] std::string field_name_acceleration_x() const { return field_name_acceleration_x_; }
        [[nodiscard]] std::string field_name_acceleration_y() const { return field_name_acceleration_y_; }
        [[nodiscard]] std::string field_name_acceleration_z() const { return field_name_acceleration_z_; }
        [[nodiscard]] std::string field_name_angular_velocity_x() const { return field_name_angular_velocity_x_; }
        [[nodiscard]] std::string field_name_angular_velocity_y() const { return field_name_angular_velocity_y_; }
        [[nodiscard]] std::string field_name_angular_velocity_z() const { return field_name_angular_velocity_z_; }
        [[nodiscard]] std::string field_name_latitude_degree() const { return field_name_latitude_degree_; }
        [[nodiscard]] std::string field_name_longitude_degree() const { return field_name_longitude_degree_; }
        [[nodiscard]] std::string field_name_altitude() const { return field_name_altitude_; }

        // publish
        void SetTrajectoryPublishPeriod(double publish_period_sec);
        void SetTfPublishPeriod(double publish_period_sec);
        void SetImuPublishPeriod(double publish_period_sec);
        void SetLidarPublishPeriod(double publish_period_sec);

        [[nodiscard]] Duration trajectory_publish_period() const { return trajectory_publish_period_; }
        [[nodiscard]] Duration tf_publish_period() const { return tf_publish_period_; }
        [[nodiscard]] Duration imu_publish_period() const { return imu_publish_period_; }
        [[nodiscard]] Duration lidar_publish_period() const { return lidar_publish_period_; }

        // write
        void SetWriteLidarDataXYZ(bool write_lidar_data_xyz) { write_lidar_data_xyz_ = write_lidar_data_xyz; };
        void SetWriteLidarDataRosbag(bool write_lidar_data_rosbag) { write_lidar_data_rosbag_ = write_lidar_data_rosbag; };
        void SetWriteDistortedCameraDataRosbag(bool write_distorted_camera_data_rosbag) { write_distorted_camera_data_rosbag_ = write_distorted_camera_data_rosbag; };
        void SetWriteRectifiedCameraDataRosbag(bool write_rectified_camera_data_rosbag) { write_rectified_camera_data_rosbag_ = write_rectified_camera_data_rosbag; };

        [[nodiscard]] bool write_lidar_data_xyz() const { return write_lidar_data_xyz_; }
        [[nodiscard]] bool write_lidar_data_rosbag() const { return write_lidar_data_rosbag_; }
        [[nodiscard]] bool write_distorted_camera_data_rosbag() const { return write_distorted_camera_data_rosbag_; }
        [[nodiscard]] bool write_rectified_camera_data_rosbag() const { return write_rectified_camera_data_rosbag_; }

        // validate
        void ValidateAndPrepare() const;

    private:
        // paths
        std::filesystem::path source_sensor_data_directory_;
        std::optional<std::filesystem::path> source_bus_signals_filepath_;
        std::optional<std::filesystem::path> source_sensor_configuration_filepath_;

        std::filesystem::path target_directory_;
        std::string target_trajectories_subdirectory_name_ = "driving_dataset_gnss_trajectories";

        // filter
        std::optional<Time> filter_start_timestamp_ = {};
        std::optional<Time> filter_stop_timestamp_ = {};
        std::set<CameraDirectionIdentifier> filter_camera_identifiers_ = {};

        // lidar sensors
        std::map<CameraDirectionIdentifier, std::map<uint64_t, uint64_t>> lidar_sensor_id_remappings_;

        // camera sensors
        std::string field_name_image_time_ = "cam_tstamp";

        double_t camera_sensors_time_estimation_lidar_timeframe_to_image_ratio_ = 0;
        bool camera_sensors_time_estimation_validate_method_ = false;
        Duration camera_sensors_time_estimation_time_tolerance_ = 0;

        // bus signals
        Duration time_offset_tolerance_ = 0;
        double_t altitude_offset_ = 0;

        std::string field_name_velocity_ = "vehicle_speed";
        std::string field_name_acceleration_x_ = "acceleration_x";
        std::string field_name_acceleration_y_ = "acceleration_y";
        std::string field_name_acceleration_z_ = "acceleration_z";
        std::string field_name_angular_velocity_x_ = "angular_velocity_omega_x";
        std::string field_name_angular_velocity_y_ = "angular_velocity_omega_y";
        std::string field_name_angular_velocity_z_ = "angular_velocity_omega_z";
        std::string field_name_latitude_degree_ = "latitude_degree";
        std::string field_name_longitude_degree_ = "longitude_degree";
        std::string field_name_altitude_ = "altitude";

        // publish
        Duration trajectory_publish_period_ = FromSeconds(50e-3);
        Duration tf_publish_period_ = FromSeconds(500e-3);
        Duration imu_publish_period_ = FromSeconds(5e-3);
        Duration lidar_publish_period_ = FromSeconds(10e-3);

        // write
        bool write_lidar_data_xyz_ = false;
        bool write_lidar_data_rosbag_ = true;
        bool write_distorted_camera_data_rosbag_ = false;
        bool write_rectified_camera_data_rosbag_ = true;
    };
}

#endif //A2D2_ROS_PREPARER_CONVERTER_OPTIONS_H
