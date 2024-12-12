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


#ifndef A2D2_ROS_PREPARER_LIDAR_SCAN_TIMESERIES_PER_VIEW_H
#define A2D2_ROS_PREPARER_LIDAR_SCAN_TIMESERIES_PER_VIEW_H

#include <filesystem>

#include "../common/types.h"
#include "timed_point_cloud_data.h"

namespace a2d2_ros_preparer {

    class LidarScanTimeseriesPerView {

    public:
        explicit LidarScanTimeseriesPerView(const std::filesystem::path& directory_path,
                                            const std::string& frame_id,
                                            const std::map<uint64_t, uint64_t>& sensor_id_remappings,
                                            std::optional<Time> filter_start_timestamp,
                                            std::optional<Time> filter_stop_timestamp);

        [[nodiscard]] bool HasData(DataSequenceId id) const;
        [[nodiscard]] std::vector<DataSequenceId> GetDataSequenceIds() const;
        std::vector<DataSequenceId> GetDataSequenceIds(Time timestamp_min, Time timestamp_max);

        [[nodiscard]] TimedPointCloudData GetTimedPointCloudData(DataSequenceId id) const;
        [[nodiscard]] TimedPointCloudData GetTimedPointCloudData(const std::set<DataSequenceId>& sequence_ids) const;
        [[nodiscard]] TimedPointCloudData GetTimedPointCloudData(Time timestamp_min, Time timestamp_max) const;

        std::string frame_id() { return frame_id_; }

        [[nodiscard]] Time GetTime(DataSequenceId sequence_id) const;
        [[nodiscard]] Time GetStartTime() const;
        [[nodiscard]] Time GetStopTime() const;
        [[nodiscard]] Duration GetTimeToRectimeOffsetMedian(const Duration& tolerance = 0) const;

        [[nodiscard]] std::map<DataSequenceId, Time> GetPointCloudTimeMins() const { return point_cloud_time_min_; };
        [[nodiscard]] std::map<DataSequenceId, Time> GetPointCloudTimeMaxs() const { return point_cloud_time_max_; };

    private:
        std::string frame_id_;
        std::map<DataSequenceId, std::filesystem::path> filepaths_;
        const std::map<uint64_t, uint64_t> sensor_id_remappings_;

        std::map<DataSequenceId, Time> point_cloud_time_min_;
        std::map<DataSequenceId, Time> point_cloud_time_max_;
        std::map<DataSequenceId, Time> point_cloud_time_minmaxmiddle_;
        std::map<DataSequenceId, Duration> point_cloud_time_rectime_offset_median_;
    };
}

#endif //A2D2_ROS_PREPARER_LIDAR_SCAN_TIMESERIES_PER_VIEW_H
