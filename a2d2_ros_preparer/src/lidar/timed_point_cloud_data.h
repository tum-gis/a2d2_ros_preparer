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


#ifndef A2D2_ROS_PREPARER_TIMED_POINT_CLOUD_DATA_H
#define A2D2_ROS_PREPARER_TIMED_POINT_CLOUD_DATA_H

#include <filesystem>
#include <Eigen/StdVector>
#include <utility>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include "../common/timing.h"
#include "../common/types.h"

namespace a2d2_ros_preparer {

    struct TimedRangefinderPoint {
        Eigen::Vector3d position;
        Time time; // timestamp from GNSS receivers (in ns)
        Time rectime; // timestamp from time master (in ns)
        uint64_t boundary;
        uint64_t sensor_id;
        uint64_t intensity;
        double azimuth;
        double col;
        double row;
        double depth;
        double distance;
        std::string dataset_view_id;
        DataSequenceId dataset_sequence_id;
    };

    class TimedPointCloudData {
    public:
        explicit TimedPointCloudData(std::vector<TimedRangefinderPoint>& ranges, std::string frame_id):
            ranges_(ranges), frame_id_(std::move(frame_id)) {};
        explicit TimedPointCloudData(std::vector<TimedPointCloudData> point_clouds);

        [[nodiscard]] std::string frame_id() const { return frame_id_; }
        [[nodiscard]] bool empty() const { return ranges_.empty(); }
        [[nodiscard]] size_t size() const { return ranges_.size(); }
        [[nodiscard]] std::vector<TimedRangefinderPoint> ranges() const { return ranges_; }
        [[nodiscard]] Time GetStartTime() const;
        [[nodiscard]] Time GetStopTime() const;
        [[nodiscard]] Time GetMinMaxMiddleTime() const;
        [[nodiscard]] Duration GetTimeRectimeOffsetMedian() const;

        pcl::PointCloud<pcl::PointXYZI> ToPcl();
        sensor_msgs::PointCloud2 ToRos();

    private:
        std::string frame_id_;
        std::vector<TimedRangefinderPoint> ranges_;
    };

    TimedPointCloudData TransformPointCloud(const TimedPointCloudData& point_cloud, const Eigen::Affine3d& affine, const std::string& target_frame_id);
    TimedPointCloudData FilterPointCloud(const TimedPointCloudData& point_cloud, Time time_min, Time time_max);
    TimedPointCloudData FilterPointCloud(const TimedPointCloudData& point_cloud, uint64_t sensor_id);
    TimedPointCloudData FilterPointDuplicates(const TimedPointCloudData& point_cloud);

    void WriteToXYZFileVerbose(const TimedPointCloudData& point_cloud, const std::filesystem::path& filepath);
    void WriteToXYZFile(const TimedPointCloudData& point_cloud, const std::filesystem::path& filepath);
}

#endif //A2D2_ROS_PREPARER_TIMED_POINT_CLOUD_DATA_H
