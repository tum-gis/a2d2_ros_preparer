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


#include "lidar_scan_stream.h"

#include <execution>
#include <utility>
#include <glog/logging.h>

#include "../common/utility.h"

namespace a2d2_ros_preparer {

    LidarScanStream::LidarScanStream(const std::filesystem::path& lidar_data_directory,
                                     const std::vector<CameraDirectionIdentifier>& camera_identifiers,
                                     std::vector<LidarDirectionIdentifier> lidar_identifiers,
                                     const std::map<CameraDirectionIdentifier, std::map<uint64_t, uint64_t>> sensor_id_remappings,
                                     std::map<CameraDirectionIdentifier, Eigen::Affine3d> camera_to_base_affine_transformation,
                                     std::map<LidarDirectionIdentifier, Eigen::Affine3d> lidar_to_base_affine_transformation,
                                     std::optional<Time> filter_start_timestamp,
                                     std::optional<Time> filter_stop_timestamp) :
            lidar_identifiers_(std::move(lidar_identifiers)),
            camera_to_base_affine_transformation_(std::move(camera_to_base_affine_transformation)),
            lidar_to_base_affine_transformation_(std::move(lidar_to_base_affine_transformation)) {

        for (auto const& current_camera_identifier: camera_identifiers) {
            if (!IsSubdirectoryContained(lidar_data_directory, current_camera_identifier)) {
                LOG(WARNING) << "Lidar data directory for direction '" << current_camera_identifier << "' not found.";
                continue;
            }
            auto directory_path = GetSubdirectoryPath(lidar_data_directory, current_camera_identifier);

            std::map<uint64_t, uint64_t> current_sensor_id_remappings = {};
            if (sensor_id_remappings.count(current_camera_identifier))
            {
                current_sensor_id_remappings = sensor_id_remappings.at(current_camera_identifier);
            }

            auto sensor_data_timeseries = LidarScanTimeseriesPerView(directory_path, current_camera_identifier, current_sensor_id_remappings, filter_start_timestamp, filter_stop_timestamp);
            lidar_scan_timeseries_per_view_.insert(std::make_pair(current_camera_identifier, sensor_data_timeseries));
        }


        auto start_timestamp = GetStartTimestamp();
        auto stop_timestamp = GetStopTimestamp();
        LOG(INFO) << std::setprecision(std::numeric_limits<double>::digits10 + 2)
                << "Completed lidar scan readin with start timestamp of " << ToSeconds(start_timestamp) << "s and "
                << "stop timestamp of " << ToSeconds(stop_timestamp) << "s";
    }


    Duration LidarScanStream::GetTimeToRectimeOffsetMedian(const Duration &tolerance) const {

        std::vector<Duration> offsets;
        for (auto& [_, current_sensor_data_timeseries] : lidar_scan_timeseries_per_view_) {
            auto current_difference = current_sensor_data_timeseries.GetTimeToRectimeOffsetMedian(tolerance);
            offsets.push_back(current_difference);
        }
        auto mean_offset = CalculateMean(offsets);

        if (tolerance != 0 && ContainsDeviationGreaterThanThreshold(offsets, mean_offset, tolerance))
            LOG(WARNING) << "Time to rectime offsets between the different views differ above the tolerance (" << tolerance << ")";

        return mean_offset;
    }

    TimedPointCloudData LidarScanStream::GetTimedPointCloudData(Time timestamp_min, Time timestamp_max) {
        auto camera_identifiers = GetCameraIdentifiers();
        std::mutex m;
        std::vector<TimedPointCloudData> point_clouds;

        std::for_each(std::execution::par_unseq, std::begin(camera_identifiers), std::end(camera_identifiers), [&](const std::string& current_camera_identifier) {
            auto point_cloud = lidar_scan_timeseries_per_view_.at(current_camera_identifier).GetTimedPointCloudData(timestamp_min, timestamp_max);
            auto transformed_point_cloud = TransformPointCloud(point_cloud, camera_to_base_affine_transformation_.at(current_camera_identifier), "base_link");

            std::lock_guard<std::mutex> guard(m);
            point_clouds.push_back(transformed_point_cloud);
        });

        return TimedPointCloudData(point_clouds);
    }

    std::vector<DataSequenceId> LidarScanStream::GetAllDataSequenceIds() const {
        std::set<DataSequenceId> unique_identifiers_set;

        for (auto& [_, current_sensor_data_timeseries] : lidar_scan_timeseries_per_view_) {

            for (auto current_ids: current_sensor_data_timeseries.GetDataSequenceIds())
                unique_identifiers_set.insert(current_ids);
        }

        std::vector<DataSequenceId> unique_identifier;
        std::copy(unique_identifiers_set.begin(), unique_identifiers_set.end(), std::back_inserter(unique_identifier));
        std::sort(unique_identifier.begin(), unique_identifier.end());

        return unique_identifier;
    }

    std::vector<DataSequenceId> LidarScanStream::GetAllDataSequenceIds(const CameraDirectionIdentifier& camera_identifier, Time start_timestamp, Time stop_timestamp) const {

        auto lidar_scan_timeseries = lidar_scan_timeseries_per_view_.at(camera_identifier);
        return lidar_scan_timeseries.GetDataSequenceIds(start_timestamp, stop_timestamp);
    }

    bool LidarScanStream::IsSensorDataAvailable(DataSequenceId id) const {
        for (const auto& [_, current_sensor_data_timeseries] : lidar_scan_timeseries_per_view_) {
            if (!current_sensor_data_timeseries.HasData(id))
                return false;
        }
        return true;
    }

    Time LidarScanStream::GetStartTimestamp() const {
        std::vector<Time> start_timestamps;
        for (auto& [_, current_sensor_data_timeseries] : lidar_scan_timeseries_per_view_)
            start_timestamps.push_back(current_sensor_data_timeseries.GetStartTime());

        return *std::max_element(start_timestamps.begin(), start_timestamps.end());
    }

    Time LidarScanStream::GetStopTimestamp() const {
        std::vector<Time> stop_timestamps;
        for (auto& [_, current_sensor_data_timeseries] : lidar_scan_timeseries_per_view_)
            stop_timestamps.push_back(current_sensor_data_timeseries.GetStopTime());

        return *std::min_element(stop_timestamps.begin(), stop_timestamps.end());
    }

    TimedPointCloudData LidarScanStream::GetTimedPointCloudData(DataSequenceId sequence_id, const std::set<std::string>& view_ids) {
        std::vector<TimedPointCloudData> point_clouds;
        for (auto & it: lidar_scan_timeseries_per_view_) {
            if (view_ids.empty() || view_ids.count(it.first) > 0) {
                auto point_cloud = it.second.GetTimedPointCloudData(sequence_id);
                auto transformed_point_cloud = TransformPointCloud(point_cloud, camera_to_base_affine_transformation_.at(it.first), "base_link");
                point_clouds.push_back(transformed_point_cloud);

            }
        }

        return TimedPointCloudData(point_clouds);
    }

    TimedPointCloudData LidarScanStream::GetTimedPointCloudData(const std::string& view_id, DataSequenceId sequence_id) {

        auto lidar_scan_timeseries = lidar_scan_timeseries_per_view_.at(view_id);
        return lidar_scan_timeseries.GetTimedPointCloudData(sequence_id);
    }

    TimedPointCloudData LidarScanStream::GetTimedPointCloudData(const std::string& view_id, const std::set<DataSequenceId>& sequence_ids) {

        auto lidar_scan_timeseries = lidar_scan_timeseries_per_view_.at(view_id);
        std::vector<TimedPointCloudData> point_clouds;
        for (auto const it : sequence_ids) {
            point_clouds.push_back(lidar_scan_timeseries.GetTimedPointCloudData(it));
        }

        return TimedPointCloudData(point_clouds);
    }

    TimedPointCloudData LidarScanStream::GetTimedPointCloudData(const std::map<std::string, std::set<DataSequenceId>>& ids) {

        std::vector<TimedPointCloudData> point_clouds;
        for (auto const& it : ids) {
            auto lidar_scan_timeseries = lidar_scan_timeseries_per_view_.at(it.first);

            for (auto const current_sequence_id: it.second) {
                auto point_cloud = lidar_scan_timeseries.GetTimedPointCloudData(current_sequence_id);
                auto transformed_point_cloud = TransformPointCloud(point_cloud, camera_to_base_affine_transformation_.at(it.first), "base_link");
                point_clouds.push_back(transformed_point_cloud);
            }
        }

        return TimedPointCloudData(point_clouds);
    }

    void LidarScanStream::WriteAllDataToRosbag(rosbag::Bag &bag, Time start_timestamp, Time stop_timestamp) {

        auto complete_point_cloud = GetTimedPointCloudData(start_timestamp, stop_timestamp);
        complete_point_cloud = FilterPointDuplicates(complete_point_cloud);

        for (auto it = lidar_identifiers_.begin(); it != lidar_identifiers_.end(); ++it) {
            int index = std::distance(lidar_identifiers_.begin(), it);

            auto current_point_cloud = FilterPointCloud(complete_point_cloud, index);
            if (!current_point_cloud.empty()) {
                auto transform = lidar_to_base_affine_transformation_.at(*it);
                auto transform_inverse = transform.inverse(Eigen::Affine);
                auto transformed_point_cloud = TransformPointCloud(current_point_cloud, transform_inverse, "lidar_" + *it);

                auto current_msg = transformed_point_cloud.ToRos();
                bag.write("/lidar_" + *it, current_msg.header.stamp, current_msg);
            }
        }

    }

    void
    LidarScanStream::WriteAllDataToXYZFile(const std::filesystem::path& directory_path, const std::string& filename, Time start_timestamp, Time stop_timestamp) {
        auto complete_point_cloud = GetTimedPointCloudData(start_timestamp, stop_timestamp);
        complete_point_cloud = FilterPointDuplicates(complete_point_cloud);

        // individual non-transformed point cloud
        auto complete_file_path = directory_path / (filename + ".xyz");
        WriteToXYZFile(complete_point_cloud, complete_file_path);

        // back-transformed point clouds differentiated by each sensor
        auto individual_directory_path = directory_path / filename;
        std::filesystem::create_directory(individual_directory_path);

        for (auto it = lidar_identifiers_.begin(); it != lidar_identifiers_.end(); ++it) {
            auto individual_file_path = individual_directory_path / (*it + ".xyz");

            int index = std::distance(lidar_identifiers_.begin(), it);

            auto current_point_cloud = FilterPointCloud(complete_point_cloud, index);
            if (!current_point_cloud.empty()) {
                auto transform = lidar_to_base_affine_transformation_.at(*it);
                auto transform_inverse = transform.inverse(Eigen::Affine);
                auto transformed_point_cloud = TransformPointCloud(current_point_cloud, transform_inverse, "lidar_" + *it);

                WriteToXYZFile(transformed_point_cloud, individual_file_path);
            }
        }
    }

    Time LidarScanStream::GetTime(const CameraDirectionIdentifier& camera_identifier, DataSequenceId sequence_id) const {
        return lidar_scan_timeseries_per_view_.at(camera_identifier).GetTime(sequence_id);
    }

    std::vector<CameraDirectionIdentifier> LidarScanStream::GetCameraIdentifiers() const {
        std::vector<CameraDirectionIdentifier> camera_identifiers;
        for(const auto& [key, _] : lidar_scan_timeseries_per_view_)
            camera_identifiers.push_back(key);
        return camera_identifiers;
    }

    std::map<CameraDirectionIdentifier, std::vector<DataSequenceId>>
    LidarScanStream::GetAllDataSequenceIdsPerCameraDirection() const {
        auto all_data_sequence_ids = std::map<CameraDirectionIdentifier, std::vector<DataSequenceId>>();
        for(const auto& [key, timeseries] : lidar_scan_timeseries_per_view_)
            all_data_sequence_ids.insert(std::make_pair(key, timeseries.GetDataSequenceIds()));

        return all_data_sequence_ids;
    }

    std::map<CameraDirectionIdentifier, std::map<DataSequenceId, Time>>
    LidarScanStream::GetPointCloudTimeMinsPerCameraDirection() const {
        auto point_cloud_time_mins_per_camera_direction = std::map<CameraDirectionIdentifier, std::map<DataSequenceId, Time>>();
        for(const auto& [key, timeseries] : lidar_scan_timeseries_per_view_)
            point_cloud_time_mins_per_camera_direction.insert(std::make_pair(key, timeseries.GetPointCloudTimeMins()));

        return point_cloud_time_mins_per_camera_direction;
    }

    std::map<CameraDirectionIdentifier, std::map<DataSequenceId, Time>>
    LidarScanStream::GetPointCloudTimeMaxsPerCameraDirection() const {
        auto point_cloud_time_mins_per_camera_direction = std::map<CameraDirectionIdentifier, std::map<DataSequenceId, Time>>();
        for(const auto& [key, timeseries] : lidar_scan_timeseries_per_view_)
            point_cloud_time_mins_per_camera_direction.insert(std::make_pair(key, timeseries.GetPointCloudTimeMaxs()));

        return point_cloud_time_mins_per_camera_direction;
    }

}
