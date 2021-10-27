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


#include "options.h"

#include <cmath>
#include <iostream>
#include <glog/logging.h>

namespace a2d2_ros_preparer {

    std::filesystem::path Options::source_bus_signals_filepath() const {
        if (source_bus_signals_filepath_.has_value())
            return source_bus_signals_filepath_.value();
        else
            return (source_sensor_data_directory() / "bus_signals.json");
    }

    std::filesystem::path Options::source_sensor_configuration_filepath() const {
        if (source_sensor_configuration_filepath_.has_value())
            return source_sensor_configuration_filepath_.value();
        else
            return (source_sensor_data_directory() / "cams_lidars.json");
    }

    void Options::SetTimeOffsetTolerance(double seconds) {
        CHECK(std::isfinite(seconds)) << "Time offset tolerance must be finite";
        time_offset_tolerance_ = FromSeconds(seconds);
    }

    void Options::SetAltitudeOffset(double offset) {
        CHECK(std::isfinite(offset)) << "Altitude offset must be finite";
        altitude_offset_ = offset;
    }

    void Options::SetFilterStartTimestamp(double start_timestamp) {
        CHECK_GE(start_timestamp, 0) << "Start timestamp must be greater than or equal zero";
        if (filter_stop_timestamp_.has_value())
            CHECK_LT(FromSeconds(start_timestamp), filter_stop_timestamp_.value()) << "Start timestamp must be before stop timestamp";

        filter_start_timestamp_ = FromSeconds(start_timestamp);
    }

    void Options::SetFilterStopTimestamp(double stop_timestamp) {
        CHECK_GE(stop_timestamp, 0) << "Stop timestamp must be greater than or equal zero";
        if (filter_start_timestamp_.has_value())
            CHECK_GE(FromSeconds(stop_timestamp), filter_start_timestamp_.value()) << "Stop timestamp must be after start timestamp";

        filter_stop_timestamp_ = FromSeconds(stop_timestamp);
    }

    void Options::SetFilterCameraIdentifiers(const std::vector<std::string>& camera_identifiers) {
        filter_camera_identifiers_ = std::set<std::string>(std::make_move_iterator(camera_identifiers.begin()),
                      std::make_move_iterator(camera_identifiers.end()));
    }

    void Options::SetCameraSensorsTimeEstimationTimeTolerance(double time_tolerance_sec) {
        CHECK_GT(time_tolerance_sec, 0) << "Sensor image time estimation tolerance must be greater than zero";
        camera_sensors_time_estimation_time_tolerance_ = FromSeconds(time_tolerance_sec);
    }

    void Options::SetTrajectoryPublishPeriod(double publish_period_sec) {
        CHECK_GT(publish_period_sec, 0) << "Publish period must be greater than zero";
        trajectory_publish_period_ = FromSeconds(publish_period_sec);
    }

    void Options::SetTfPublishPeriod(double publish_period_sec) {
        CHECK_GT(publish_period_sec, 0) << "Publish period must be greater than zero";
        tf_publish_period_ = FromSeconds(publish_period_sec);
    }

    void Options::SetImuPublishPeriod(double publish_period_sec) {
        CHECK_GT(publish_period_sec, 0) << "Publish period must be greater than zero";
        imu_publish_period_ = FromSeconds(publish_period_sec);
    }

    void Options::SetLidarPublishPeriod(double publish_period_sec) {
        CHECK_GT(publish_period_sec, 0) << "Publish period must be greater than zero";
        lidar_publish_period_ = FromSeconds(publish_period_sec);
    }

    std::optional<Duration> Options::camera_sensors_time_estimation_time_tolerance() const {
        if (!camera_sensors_time_estimation_validate_method_ || camera_sensors_time_estimation_time_tolerance_ == 0)
            return {};
        return camera_sensors_time_estimation_time_tolerance_;
    }

    void Options::ValidateAndPrepare() const {
        CHECK(exists(source_sensor_data_directory())) << "Source sensor data directory does not exist at: " << source_sensor_data_directory();
        CHECK(exists(source_bus_signals_filepath())) << "Source bus signals file does not exist at: " << source_bus_signals_filepath();
        CHECK(exists(source_sensor_configuration_filepath())) << "Source sensor configuration file does not exist at: " << source_sensor_configuration_filepath();

        CHECK(!target_directory().empty()) << "Target directory must be configured and not be empty.";
        CHECK(exists(target_directory().parent_path())) << "Parent path of configured target directory must exist for: ." << target_directory();
        if (!exists(target_directory())) {
            LOG(INFO) << "Creating target directory at: " << target_directory();
            std::filesystem::create_directory(target_directory());
        }

    }

}
