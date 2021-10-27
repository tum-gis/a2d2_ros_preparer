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


#ifndef A2D2_ROS_PREPARER_BUS_SIGNALS_TIMESERIES_H
#define A2D2_ROS_PREPARER_BUS_SIGNALS_TIMESERIES_H

#include <filesystem>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nlohmann/json.hpp>
#include <limits>
#include "../options.h"
#include "../common/value_timeseries.h"
#include "../common/timing.h"

namespace a2d2_ros_preparer {

    class BusSignalStream {
    public:
        explicit BusSignalStream(const Options& options, const Duration& time_offset);

        sensor_msgs::Imu GetImuMessage(Time time);
        sensor_msgs::NavSatFix GetNavSatMessage(Time time);
        std::vector<nav_msgs::Odometry> GetOdometryMessage(Time start_timestamp, Time stop_timestamp, Duration time_delta);

        Time GetStartTime();
        Time GetStopTime();

        void WriteToGeojsonFile(const std::filesystem::path& file_path,
                                Duration time_delta,
                                std::optional<Time> start_time = {},
                                std::optional<Time> stop_time = {});

    private:
        nlohmann::json bus_data_json_;

        ValueTimeseries<double> velocity_;

        ValueTimeseries<double> acceleration_x_;
        ValueTimeseries<double> acceleration_y_;
        ValueTimeseries<double> acceleration_z_;

        ValueTimeseries<double> angular_velocity_x_;
        ValueTimeseries<double> angular_velocity_y_;
        ValueTimeseries<double> angular_velocity_z_;

        ValueTimeseries<double> latitude_degree_;
        ValueTimeseries<double> longitude_degree_;
        ValueTimeseries<double> altitude_;

        static ValueTimeseries<double> LoadTimeseriesFromJSON(const nlohmann::json& json_data, const std::string& field_name, Duration time_offset, double value_offset = 0.0, double value_factor = 1.0);
    };
}

#endif //A2D2_ROS_PREPARER_BUS_SIGNALS_TIMESERIES_H
