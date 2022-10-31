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


#include "timed_point_cloud_data.h"

#include <glog/logging.h>
#include "../common/utility.h"

namespace a2d2_ros_preparer {

    sensor_msgs::PointCloud2 TimedPointCloudData::ToRos() {

        sensor_msgs::PointCloud2 ros_point_cloud_msg;

        auto pcl_point_cloud = ToPcl();
        pcl::toROSMsg(pcl_point_cloud, ros_point_cloud_msg);

        ros_point_cloud_msg.header.frame_id = frame_id_;
        ros_point_cloud_msg.header.stamp = ToRosTime(GetMinMaxMiddleTime());
        return ros_point_cloud_msg;
    }

    pcl::PointCloud<pcl::PointXYZI> TimedPointCloudData::ToPcl() {

        pcl::PointCloud<pcl::PointXYZI> pcl_point_cloud;
        pcl_point_cloud.height = 1;
        pcl_point_cloud.width = size();

        for (const auto &timed_point : ranges_) {
            pcl::PointXYZI pcl_point = pcl::PointXYZI();
            pcl_point.x = static_cast<float>(timed_point.position.x());
            pcl_point.y = static_cast<float>(timed_point.position.y());
            pcl_point.z = static_cast<float>(timed_point.position.z());
            pcl_point.intensity = static_cast<float>(timed_point.intensity);

            pcl_point_cloud.push_back(pcl_point);
        }

        return pcl_point_cloud;
    }

    Time TimedPointCloudData::GetStartTime() const {
        return std::min_element(ranges_.begin(),
                                ranges_.end(),
                                [](const TimedRangefinderPoint& x, const TimedRangefinderPoint& y) { return x.time < y.time; })->time;
    }

    Time TimedPointCloudData::GetStopTime() const {
        return std::max_element(ranges_.begin(),
                                ranges_.end(),
                                [](const TimedRangefinderPoint& x, const TimedRangefinderPoint& y) { return x.time < y.time; })->time;
    }

    Time TimedPointCloudData::GetMinMaxMiddleTime() const {
        auto min_time = GetStartTime();
        auto max_time = GetStopTime();
        auto min_max_duration = max_time - min_time;
        return min_time + min_max_duration/2;
    }

    Duration TimedPointCloudData::GetTimeRectimeOffsetMedian() const {
        std::vector<Duration> offsets;
        for (const auto& current_range: ranges()) {
            auto offset = static_cast<Duration>(current_range.time - current_range.rectime);
            offsets.push_back(offset);
        }
        return GetMedian(offsets);
    }

    TimedPointCloudData::TimedPointCloudData(std::vector<TimedPointCloudData> point_clouds) {
        CHECK(!point_clouds.empty()) << "Point clouds must not be empty";

        frame_id_ = point_clouds.front().frame_id_;
        for (auto current_point_cloud : point_clouds) {
            CHECK_EQ(current_point_cloud.frame_id_, frame_id_) << "All point clouds must have the same frame id.";
            ranges_.insert(ranges_.end(), current_point_cloud.ranges_.begin(), current_point_cloud.ranges_.end());
        }
    }

    TimedPointCloudData TransformPointCloud(const TimedPointCloudData& point_cloud, const Eigen::Affine3d& affine, const std::string& target_frame_id) {

        std::vector<TimedRangefinderPoint> transformed_ranges;
        for (const auto& current_range: point_cloud.ranges()) {
            TimedRangefinderPoint result_range = current_range;
            result_range.position = affine * current_range.position;

            transformed_ranges.push_back(result_range);
        }
        TimedPointCloudData transformed_point_cloud = TimedPointCloudData(transformed_ranges, target_frame_id);
        return transformed_point_cloud;
    }

    TimedPointCloudData FilterPointCloud(const TimedPointCloudData& point_cloud, Time time_min, Time time_max) {
        std::vector<TimedRangefinderPoint> filtered_ranges;
        for (const auto& current_range: point_cloud.ranges()) {
            if (time_min <= current_range.time && current_range.time <= time_max)
                filtered_ranges.push_back(current_range);
        }

        return TimedPointCloudData(filtered_ranges, point_cloud.frame_id());
    }

    TimedPointCloudData FilterPointCloud(const TimedPointCloudData& point_cloud, uint64_t sensor_id) {
        std::vector<TimedRangefinderPoint> filtered_ranges;
        for (const auto& current_range: point_cloud.ranges()) {
            if (current_range.sensor_id == sensor_id)
                filtered_ranges.push_back(current_range);
        }
        return TimedPointCloudData(filtered_ranges, point_cloud.frame_id());
    }

    TimedPointCloudData FilterPointDuplicates(const TimedPointCloudData& point_cloud) {
        std::map<uint64_t, DataSequenceId> most_recent_record;
        for (const auto& current_range: point_cloud.ranges()) {
            if (most_recent_record.count(current_range.time) == 0 || current_range.dataset_sequence_id < most_recent_record[current_range.time])
                most_recent_record[current_range.time] = current_range.dataset_sequence_id;

        }

        std::vector<TimedRangefinderPoint> filtered_ranges;
        for (const auto& current_range: point_cloud.ranges()) {
            if (most_recent_record[current_range.time] == current_range.dataset_sequence_id )
                filtered_ranges.push_back(current_range);
        }

        return TimedPointCloudData(filtered_ranges, point_cloud.frame_id());
    }


    void WriteToXYZFileVerbose(const TimedPointCloudData& point_cloud, const std::filesystem::path& filepath) {

        auto start_time = point_cloud.GetStartTime();

        std::ofstream file;
        file.open (filepath);
        file << "X Y Z intensity time time_since_start_in_ms timestamp rectime boundary azimuth col row depth distance sensor_id dataset_view_id dataset_sequence_id\n";
        for (const auto& range: point_cloud.ranges()) {
            file << std::setprecision(std::numeric_limits<double>::digits10 + 2) <<
                 range.position.x() << " " <<
                 range.position.y() << " " <<
                 range.position.z() << " " <<
                 range.intensity << " " <<
                 range.time << " " <<
                 static_cast<double>(range.time-start_time)/1e6 << " " <<
                 range.rectime << " " <<
                 range.boundary << " " <<
                 range.azimuth << " " <<
                 range.col << " " <<
                 range.row << " " <<
                 range.depth << " " <<
                 range.distance << " " <<
                 range.sensor_id << " " <<
                 range.dataset_view_id << " " <<
                 range.dataset_sequence_id << "" <<
                 "\n";
        }
        file.close();
    }

    void WriteToXYZFile(const TimedPointCloudData& point_cloud, const std::filesystem::path& filepath) {

        auto start_time = point_cloud.GetStartTime();

        std::ofstream file;
        file.open (filepath);
        file << "X Y Z intensity time time_since_start_in_ms\n";
        for (const auto& range: point_cloud.ranges()) {
            file << std::setprecision(std::numeric_limits<double>::digits10 + 2) <<
                 range.position.x() << " " <<
                 range.position.y() << " " <<
                 range.position.z() << " " <<
                 range.intensity << " " <<
                 range.time << " " <<
                 static_cast<double>(range.time-start_time)/1e6 << " " <<
                 "\n";
        }
        file.close();
    }
}
