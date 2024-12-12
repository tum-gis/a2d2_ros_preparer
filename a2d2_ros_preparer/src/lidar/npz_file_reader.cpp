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


#include "npz_file_reader.h"

#include <cnpy/cnpy.h>
#include <glog/logging.h>

namespace a2d2_ros_preparer {

    TimedPointCloudData ReadFile(const std::filesystem::path& filepath, const std::string& view_id, const DataSequenceId& sequence_id, const std::map<uint64_t, uint64_t>& sensor_id_remappings) {
        CHECK(exists(filepath)) << "File not found at: " << filepath;

        cnpy::npz_t npz_file = cnpy::npz_load(filepath);
        auto number_points = npz_file["pcloud_points"].shape[0];

        auto position_data = npz_file["pcloud_points"].as_vec<double>();
        auto timestamp_data = npz_file["pcloud_attr.timestamp"].as_vec<uint64_t>();
        auto rectime_data = npz_file["pcloud_attr.rectime"].as_vec<uint64_t>();
        auto boundary_data = npz_file["pcloud_attr.boundary"].as_vec<uint64_t>();
        auto intensity_data = npz_file["pcloud_attr.reflectance"].as_vec<uint64_t>();

        auto azimuth_data = npz_file["pcloud_attr.azimuth"].as_vec<double>();
        auto col_data = npz_file["pcloud_attr.col"].as_vec<double>();
        auto row_data = npz_file["pcloud_attr.row"].as_vec<double>();
        auto depth_data = npz_file["pcloud_attr.depth"].as_vec<double>();
        auto distance_data = npz_file["pcloud_attr.distance"].as_vec<double>();

        auto sensor_id_data = npz_file["pcloud_attr.lidar_id"].as_vec<uint64_t>();
        // remapping the lidar sensor ids in case they have been provided inconsistently per camera view
        if (!sensor_id_remappings.empty()) {
            std::vector<uint64_t> remapped_sensor_id_data;
            remapped_sensor_id_data.reserve(sensor_id_data.size());

            for (const auto id : sensor_id_data) {
                if (sensor_id_remappings.count(id) == 1) {
                    remapped_sensor_id_data.push_back(sensor_id_remappings.at(id));
                } else {
                    remapped_sensor_id_data.push_back(id); // Retain original if no mapping exists
                }
            }

            sensor_id_data = std::move(remapped_sensor_id_data);
        }
        

        std::vector<TimedRangefinderPoint> ranges;
        for (int i=0; i < number_points; i++)
        {
            TimedRangefinderPoint timed_point;
            timed_point.position = Eigen::Vector3d(position_data[(i*3)], position_data[(i*3)+1], position_data[(i*3)+2]);
            timed_point.time = MillisecondsToNanoseconds(timestamp_data[i]);

            timed_point.rectime = MillisecondsToNanoseconds(rectime_data[i]);
            timed_point.boundary = boundary_data[i];
            timed_point.intensity = intensity_data[i];

            timed_point.azimuth = azimuth_data[i];
            timed_point.col = col_data[i];
            timed_point.row = row_data[i];
            timed_point.depth = depth_data[i];
            timed_point.distance = distance_data[i];

            timed_point.sensor_id = sensor_id_data[i];
            timed_point.dataset_view_id = view_id;
            timed_point.dataset_sequence_id = sequence_id;

            ranges.push_back(timed_point);
        }

        return TimedPointCloudData(ranges, view_id);
    }

}
