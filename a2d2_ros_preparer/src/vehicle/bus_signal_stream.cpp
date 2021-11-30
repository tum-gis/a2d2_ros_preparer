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


#include "bus_signal_stream.h"

#include <glog/logging.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <tf2_eigen/tf2_eigen.h>
#include <nav_msgs/Odometry.h>


namespace a2d2_ros_preparer {

    BusSignalStream::BusSignalStream(const Options& options, const Duration& time_offset) {

        LOG(INFO) << "Reading bus data from file: " << options.source_bus_signals_filepath() << " with time offset " << ToSeconds(time_offset) << "s";

        std::ifstream jsonFile(options.source_bus_signals_filepath());
        jsonFile >> bus_data_json_;


        velocity_ = LoadTimeseriesFromJSON(bus_data_json_, options.field_name_velocity(), time_offset, 0, KmhToMps(1));
        acceleration_x_ = LoadTimeseriesFromJSON(bus_data_json_, options.field_name_acceleration_x(), time_offset, 0);
        acceleration_y_ = LoadTimeseriesFromJSON(bus_data_json_, options.field_name_acceleration_y(), time_offset, 0);
        acceleration_z_ = LoadTimeseriesFromJSON(bus_data_json_, options.field_name_acceleration_z(), time_offset, 0);

        angular_velocity_x_ = LoadTimeseriesFromJSON(bus_data_json_, options.field_name_angular_velocity_x(), time_offset, 0, DegToRad(1));
        angular_velocity_y_ = LoadTimeseriesFromJSON(bus_data_json_, options.field_name_angular_velocity_y(), time_offset, 0, DegToRad(1));
        angular_velocity_z_ = LoadTimeseriesFromJSON(bus_data_json_, options.field_name_angular_velocity_z(), time_offset, 0, DegToRad(1));

        latitude_degree_ = LoadTimeseriesFromJSON(bus_data_json_, options.field_name_latitude_degree(), time_offset, 0);
        longitude_degree_ = LoadTimeseriesFromJSON(bus_data_json_, options.field_name_longitude_degree(), time_offset, 0);
        if (bus_data_json_.contains(options.field_name_altitude()))
            altitude_ = LoadTimeseriesFromJSON(bus_data_json_, options.field_name_altitude(), time_offset, options.altitude_offset());
        else
            altitude_ = ValueTimeseries<double>();
        altitude_.SetDefaultValue(options.altitude_offset());
    }

    sensor_msgs::Imu BusSignalStream::GetImuMessage(Time time) {

        sensor_msgs::Imu imu_message;
        imu_message.header.frame_id = "imu_link";
        imu_message.header.stamp = ToRosTime(time);

        imu_message.linear_acceleration.x = acceleration_x_.GetValue(time);
        imu_message.linear_acceleration.y = acceleration_y_.GetValue(time);
        imu_message.linear_acceleration.z = acceleration_z_.GetValue(time);

        imu_message.angular_velocity.x = angular_velocity_x_.GetValue(time);
        imu_message.angular_velocity.y = angular_velocity_y_.GetValue(time);
        imu_message.angular_velocity.z = angular_velocity_z_.GetValue(time);

        return imu_message;
    }

    sensor_msgs::NavSatFix BusSignalStream::GetNavSatMessage(Time time) {

        auto nav_message = sensor_msgs::NavSatFix();
        nav_message.header.frame_id = "imu_link";
        nav_message.header.stamp = ToRosTime(time);

        nav_message.latitude = latitude_degree_.GetValue(time);
        nav_message.longitude = longitude_degree_.GetValue(time);
        nav_message.altitude = altitude_.GetValue(time);

        return nav_message;
    }

    std::vector<nav_msgs::Odometry> BusSignalStream::GetOdometryMessage(Time start_timestamp, Time stop_timestamp, Duration time_delta) {

        Eigen::Vector3d previous_pose = Eigen::Vector3d::Zero();
        Eigen::Quaterniond previous_orientation = Eigen::Quaterniond::Identity();

        std::vector<nav_msgs::Odometry> odometry_messages;

        Time current_timestamp = start_timestamp;
        while (current_timestamp < stop_timestamp) {

            auto current_angular_velocity = Eigen::Vector3d { angular_velocity_x_.GetValue(current_timestamp),
                                                              angular_velocity_y_.GetValue(current_timestamp),
                                                              angular_velocity_z_.GetValue(current_timestamp) };
            auto current_orientation_step_euler = current_angular_velocity * ToSeconds(time_delta);
            Eigen::Quaterniond current_orientation_step_quaternions =
                    Eigen::AngleAxisd(current_orientation_step_euler.x(), Eigen::Vector3d::UnitX()) *
                    Eigen::AngleAxisd(current_orientation_step_euler.y(), Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(current_orientation_step_euler.z(), Eigen::Vector3d::UnitZ());
            Eigen::Quaterniond current_orientation = previous_orientation * current_orientation_step_quaternions;
            Eigen::Vector3d current_velocity = current_orientation.toRotationMatrix() * Eigen::Vector3d(velocity_.GetValue(current_timestamp), 0, 0);
            Eigen::Vector3d current_position = previous_pose + current_velocity * ToSeconds(time_delta);


            auto current_odometry_message = nav_msgs::Odometry();
            current_odometry_message.header.frame_id = "odom";
            current_odometry_message.header.stamp = ToRosTime(current_timestamp);

            // set the velocity
            current_odometry_message.child_frame_id = "base_link";
            current_odometry_message.twist.twist.linear.x = current_velocity.x();
            current_odometry_message.twist.twist.linear.y = current_velocity.y();
            current_odometry_message.twist.twist.linear.z = current_velocity.z();
            current_odometry_message.twist.twist.angular.x = current_angular_velocity.x();
            current_odometry_message.twist.twist.angular.y = current_angular_velocity.y();
            current_odometry_message.twist.twist.angular.z = current_angular_velocity.z();

            // set the position
            current_odometry_message.pose.pose.position = tf2::toMsg(current_position);
            current_odometry_message.pose.pose.orientation = tf2::toMsg(current_orientation);
            odometry_messages.push_back(current_odometry_message);

            // jump to next timestamp
            current_timestamp = current_timestamp + time_delta;
            previous_pose = current_position;
            previous_orientation = current_orientation;
        }

        return odometry_messages;
    }

    Time BusSignalStream::GetStartTime() {
        std::vector<uint64_t> v {
                acceleration_x_.GetStartTime(),
                acceleration_y_.GetStartTime(),
                acceleration_z_.GetStartTime(),
                angular_velocity_x_.GetStartTime(),
                angular_velocity_y_.GetStartTime(),
                angular_velocity_z_.GetStartTime(),
                latitude_degree_.GetStartTime(),
                longitude_degree_.GetStartTime(),
                altitude_.GetStartTime(),
        };
        return *std::max_element(v.begin(), v.end());
    }

    Time BusSignalStream::GetStopTime() {
        std::vector<uint64_t> v {
                acceleration_x_.GetStopTime(),
                acceleration_y_.GetStopTime(),
                acceleration_z_.GetStopTime(),
                angular_velocity_x_.GetStopTime(),
                angular_velocity_y_.GetStopTime(),
                angular_velocity_z_.GetStopTime(),
                latitude_degree_.GetStopTime(),
                longitude_degree_.GetStopTime(),
                altitude_.GetStopTime(),
        };
        return *std::min_element(v.begin(), v.end());
    }

    void BusSignalStream::WriteToGeojsonFile(const std::filesystem::path& file_path, Duration time_delta, std::optional<Time> start_time, std::optional<Time> stop_time) {
        Time adjusted_start_time = GetStartTime();
        if (start_time.has_value()) {
            CHECK_LE(GetStartTime(), start_time.value()) << "Start time (" << ToSeconds(start_time.value()) << ") must be within domain of bus time.";
            CHECK_LE(start_time.value(), GetStopTime()) << "Start time (" << ToSeconds(start_time.value()) << ") must be within domain of bus time.";
            adjusted_start_time = start_time.value();
        }
        auto adjusted_stop_time = GetStopTime();
        if (stop_time.has_value()) {
            CHECK_LE(GetStartTime(), stop_time.value()) << "Stop time (" << ToSeconds(stop_time.value()) << ") must be within domain of bus time.";
            CHECK_LE(stop_time.value(), GetStopTime()) << "Stop time (" << ToSeconds(stop_time.value()) << ") must be within domain of bus time.";
            adjusted_stop_time = stop_time.value();
        }
        if (start_time.has_value() && stop_time.has_value()) {
            CHECK_LT(start_time.value(), stop_time.value());
        }
        CHECK_GT(time_delta, 0) << "Time delta duration must greater than zero.";

        LOG(INFO) << "Writing geojson file to " << file_path;

        auto current_time = adjusted_start_time;

        nlohmann::json geojson = {
                {"type", "FeatureCollection"},
                {"features", {}}
        };

        while (current_time < adjusted_stop_time) {
            nlohmann::json geojson_feature = {
                {"type", "Feature"},
                {"properties", {
                        {"timestamp_ns", current_time},
                        {"timestamp_s", ToSeconds(current_time)},
                        {"latitude_degree", latitude_degree_.GetValue(current_time)},
                        {"longitude_degree", longitude_degree_.GetValue(current_time)},
                        {"altitude", altitude_.GetValue(current_time)},
                        {"velocity", velocity_.GetValue(current_time)},
                        {"angular_velocity_x", angular_velocity_x_.GetValue(current_time)},
                        {"angular_velocity_y", angular_velocity_y_.GetValue(current_time)},
                        {"angular_velocity_z", angular_velocity_z_.GetValue(current_time)},
                }},
                {"geometry", {
                        {"type", "Point"},
                        {"coordinates", {
                            longitude_degree_.GetValue(current_time),
                            latitude_degree_.GetValue(current_time),
                            altitude_.GetValue(current_time)}},
                }},
            };
            geojson["features"].push_back(geojson_feature);

            current_time += time_delta;
        }

        std::filesystem::create_directory(file_path.parent_path());
        std::ofstream output(file_path);
        output << std::setprecision(std::numeric_limits<double>::digits10 + 2);
        output << std::setw(4) << geojson << std::endl;
    }

    ValueTimeseries<double> BusSignalStream::LoadTimeseriesFromJSON(const nlohmann::json& json_data, const std::string& field_name, Duration time_offset, double value_offset, double value_factor) {
        CHECK(json_data.contains(field_name)) << "No bus signals found for " << field_name;

        return LoadFromJSON<double>(json_data[field_name]["values"], time_offset, value_offset, value_factor);
    }

}
