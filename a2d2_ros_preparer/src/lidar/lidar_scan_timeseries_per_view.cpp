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


#include "lidar_scan_timeseries_per_view.h"

#include <execution>
#include <iostream>
#include <algorithm>
#include <filesystem>
#include <unordered_map>
#include <glog/logging.h>
#include "npz_file_reader.h"
#include "../common/utility.h"

namespace a2d2_ros_preparer {

    LidarScanTimeseriesPerView::LidarScanTimeseriesPerView(const std::filesystem::path& directory_path,
                                                           const std::string& frame_id,
                                                           std::optional<Time> filter_start_timestamp,
                                                           std::optional<Time> filter_stop_timestamp): frame_id_(frame_id) {
        LOG(INFO) << "Reading lidar data for view: " << frame_id;

        // get all filepaths
        std::vector<std::filesystem::path> all_filepaths;
        for (const auto& entry : std::filesystem::directory_iterator(directory_path)) {
            all_filepaths.push_back(entry);
        }
        std::sort(all_filepaths.begin(), all_filepaths.end());

        // find file which starts just before filter_start_timestamp
        auto iterator_begin = all_filepaths.begin();
        if (filter_start_timestamp.has_value()) {
            iterator_begin = std::lower_bound(
                    all_filepaths.begin(), all_filepaths.end(), filter_start_timestamp.value(),
                    [](const std::filesystem::path& filepath, Time time) {
                        auto pointcloud = ReadFile(filepath, "", 0);
                        return pointcloud.GetStartTime() < time;
                    });

            if (iterator_begin != all_filepaths.begin())
                std::advance(iterator_begin, -1);
        }

        // find file which stops just after filter_stop_timestamp
        auto iterator_end = all_filepaths.end();
        if (filter_stop_timestamp.has_value()) {
            iterator_end = std::upper_bound(
                    all_filepaths.begin(), all_filepaths.end(), filter_stop_timestamp.value(),
                    [](Time time, const std::filesystem::path& filepath) {
                        auto pointcloud = ReadFile(filepath, "", 0);
                        return time < pointcloud.GetStopTime();
                    });
            if (iterator_end != all_filepaths.end())
                std::advance(iterator_end, 1);
        }

        // collect filtered filepaths
        std::vector<std::filesystem::path> filtered_filepaths;
        for(auto it = iterator_begin; it != iterator_end; ++it)
            filtered_filepaths.push_back(*it);

        // start indexing
        std::mutex m;
        std::for_each(std::execution::par_unseq, std::begin(filtered_filepaths), std::end(filtered_filepaths), [&](std::filesystem::path& current_filepath) {
            DataSequenceId current_id = ExtractLastIntegerFromString(current_filepath.stem());
            auto point_cloud = ReadFile(current_filepath, frame_id_, current_id);

            auto time_min = point_cloud.GetStartTime();
            auto time_max = point_cloud.GetStopTime();
            auto time_minmaxmiddle = point_cloud.GetMinMaxMiddleTime();
            auto time_rectime_offset_median = point_cloud.GetTimeRectimeOffsetMedian();

            std::lock_guard<std::mutex> guard(m);
            filepaths_[current_id] = current_filepath;
            point_cloud_time_min_[current_id] = time_min;
            point_cloud_time_max_[current_id] = time_max;
            point_cloud_time_minmaxmiddle_[current_id] = time_minmaxmiddle;
            point_cloud_time_rectime_offset_median_[current_id] = time_rectime_offset_median;
        });

        CHECK(!filepaths_.empty()) << "Lidar scan timeseries does not contain any files";

        LOG(INFO) << std::fixed << std::setprecision(15) << "Completed reading of lidar data: (" << filepaths_.size() <<
        " files) for view (" << frame_id << ") with ids [" << filepaths_.begin()->first  << ", " <<
        filepaths_.rbegin()->first << "] with time in [" << ToUniversalSeconds(GetStartTime()) << ", "
        << ToUniversalSeconds(GetStopTime()) << "]";
    }

    TimedPointCloudData LidarScanTimeseriesPerView::GetTimedPointCloudData(DataSequenceId id) const {
        CHECK(HasData(id)) << "No lidar data for view '" << frame_id_ << "' and id '" << id << "'";

        return ReadFile(filepaths_.at(id), frame_id_, id);
    }

    std::vector<DataSequenceId> LidarScanTimeseriesPerView::GetDataSequenceIds() const {
        std::vector<DataSequenceId> data_sequence_ids;
        for (const auto& [key, _] : filepaths_)
            data_sequence_ids.push_back(key);

        //std::sort (this->filepaths_.begin(), this->filepaths_.end());
        return data_sequence_ids;
    }

    std::vector<DataSequenceId> LidarScanTimeseriesPerView::GetDataSequenceIds(Time timestamp_min, Time timestamp_max) {
        std::vector<DataSequenceId> ids = GetDataSequenceIds();

        std::vector<DataSequenceId> selected_ids_with_time;
        for (auto current_id : ids) {
            if (timestamp_min <= this->point_cloud_time_max_[current_id] && this->point_cloud_time_min_[current_id] <= timestamp_max)
                selected_ids_with_time.push_back(current_id);
        }

        return selected_ids_with_time;
    }

    bool LidarScanTimeseriesPerView::HasData(DataSequenceId id) const {
        return filepaths_.count(id) > 0;
    }

    TimedPointCloudData
    LidarScanTimeseriesPerView::GetTimedPointCloudData(Time timestamp_min, Time timestamp_max) const {
        std::vector<DataSequenceId> ids = GetDataSequenceIds();

        std::vector<TimedPointCloudData> selected_point_clouds;
        for (auto current_id : ids) {
            if (timestamp_min <= this->point_cloud_time_max_.at(current_id) && this->point_cloud_time_min_.at(current_id) <= timestamp_max)
                selected_point_clouds.push_back(GetTimedPointCloudData(current_id));
        }

        auto complete_point_cloud = TimedPointCloudData(selected_point_clouds);
        return FilterPointCloud(complete_point_cloud, timestamp_min, timestamp_max);
    }

    Time LidarScanTimeseriesPerView::GetStartTime() const {
        return min_element(point_cloud_time_min_.begin(), point_cloud_time_min_.end(),
                           [](const auto& l, const auto& r) { return l.second < r.second; })->second;
    }

    Time LidarScanTimeseriesPerView::GetStopTime() const {
        return max_element(point_cloud_time_max_.begin(), point_cloud_time_max_.end(),
                           [](const auto& l, const auto& r) { return l.second < r.second; })->second;
    }

    TimedPointCloudData LidarScanTimeseriesPerView::GetTimedPointCloudData(const std::set<DataSequenceId>& sequence_ids) const {
        std::vector<TimedPointCloudData> point_clouds;
        for (auto const current_id : sequence_ids) {
            point_clouds.push_back(GetTimedPointCloudData(current_id));
        }

        return TimedPointCloudData(point_clouds);
    }

    Time LidarScanTimeseriesPerView::GetTime(const DataSequenceId sequence_id) const {
        return point_cloud_time_minmaxmiddle_.at(sequence_id);
    }

    Duration LidarScanTimeseriesPerView::GetTimeToRectimeOffsetMedian(const Duration& tolerance) const {
        std::vector<Duration> time_to_rectime_offsets;
        for(const auto& [_, offset] : point_cloud_time_rectime_offset_median_)
            time_to_rectime_offsets.push_back(offset);

        auto median_offset = GetMedian(time_to_rectime_offsets);

        if (tolerance != 0 && ContainsDeviationGreaterThanThreshold(time_to_rectime_offsets, median_offset, tolerance))
            LOG(WARNING) << "Point clouds of frame '" << frame_id_ <<
            "' contain time offsets (time vs. rectime) that are above the tolerance (" << tolerance << ")";

        return median_offset;
    }

}
