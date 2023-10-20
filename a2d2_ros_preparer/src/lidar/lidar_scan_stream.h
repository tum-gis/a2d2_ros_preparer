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


#ifndef A2D2_ROS_PREPARER_LIDAR_SCAN_STREAM_H
#define A2D2_ROS_PREPARER_LIDAR_SCAN_STREAM_H

#include <string>
#include <map>
#include <set>
#include <rosbag/bag.h>

#include "../common/timing.h"
#include "lidar_scan_timeseries_per_view.h"

namespace a2d2_ros_preparer {

    class LidarScanStream {

    public:
        explicit LidarScanStream(const std::filesystem::path& lidar_data_directory,
                                 const std::vector<CameraDirectionIdentifier>& camera_identifiers,
                                 std::vector<LidarDirectionIdentifier> lidar_identifiers,
                                 std::map<CameraDirectionIdentifier, Eigen::Affine3d> camera_to_base_affine_transformation,
                                 std::map<LidarDirectionIdentifier, Eigen::Affine3d> lidar_to_base_affine_transformation,
                                 std::optional<Time> filter_start_timestamp,
                                 std::optional<Time> filter_stop_timestamp);

        [[nodiscard]] Time GetStartTimestamp() const;
        [[nodiscard]] Time GetStopTimestamp() const;
        [[nodiscard]] Time GetTime(const CameraDirectionIdentifier& camera_identifier, DataSequenceId sequence_id) const;
        [[nodiscard]] Duration GetTimeToRectimeOffsetMedian(const Duration& tolerance = 0) const;

        [[nodiscard]] std::vector<LidarDirectionIdentifier> GetLidarIdentifiers() const { return lidar_identifiers_; }
        [[nodiscard]] std::vector<CameraDirectionIdentifier> GetCameraIdentifiers() const;
        [[nodiscard]] std::vector<DataSequenceId> GetAllDataSequenceIds() const;
        [[nodiscard]] std::vector<DataSequenceId> GetAllDataSequenceIds(const CameraDirectionIdentifier& camera_identifier, Time start_timestamp, Time stop_timestamp) const;
        [[nodiscard]] std::map<CameraDirectionIdentifier, std::map<DataSequenceId, Time>> GetPointCloudTimeMinsPerCameraDirection() const;
        [[nodiscard]] std::map<CameraDirectionIdentifier, std::map<DataSequenceId, Time>> GetPointCloudTimeMaxsPerCameraDirection() const;
        [[nodiscard]] std::map<CameraDirectionIdentifier, std::vector<DataSequenceId>> GetAllDataSequenceIdsPerCameraDirection() const;

        [[nodiscard]] bool IsSensorDataAvailable(DataSequenceId id) const;

        void WriteAllDataToRosbag(rosbag::Bag &bag, Time start_timestamp, Time stop_timestamp);
        void WriteAllDataToXYZFile(const std::filesystem::path& directory_path, const std::string& filename, Time start_timestamp, Time stop_timestamp);

        TimedPointCloudData GetTimedPointCloudData(const std::map<std::string, std::set<DataSequenceId>>& ids);
        TimedPointCloudData GetTimedPointCloudData(DataSequenceId sequence_id, const std::set<std::string>& view_ids = std::set<std::string>());
        TimedPointCloudData GetTimedPointCloudData(const std::string& view_id, DataSequenceId sequence_id);
        TimedPointCloudData GetTimedPointCloudData(const std::string& view_id, const std::set<DataSequenceId>& sequence_ids = std::set<DataSequenceId>());
        TimedPointCloudData GetTimedPointCloudData(Time timestamp_min, Time timestamp_max);

    private:
        std::vector<LidarDirectionIdentifier> lidar_identifiers_;
        std::map<CameraDirectionIdentifier, LidarScanTimeseriesPerView> lidar_scan_timeseries_per_view_;

        std::map<CameraDirectionIdentifier, Eigen::Affine3d> camera_to_base_affine_transformation_;
        std::map<LidarDirectionIdentifier, Eigen::Affine3d> lidar_to_base_affine_transformation_;
    };
}


#endif //A2D2_ROS_PREPARER_LIDAR_SCAN_STREAM_H
