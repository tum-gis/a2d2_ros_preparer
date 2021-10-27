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


#ifndef A2D2_ROS_PREPARER_CAMERA_IMAGE_TIMESERIES_PER_VIEW_H
#define A2D2_ROS_PREPARER_CAMERA_IMAGE_TIMESERIES_PER_VIEW_H

#include <filesystem>
#include <map>
#include <sensor_msgs/Image.h>
#include "../common/types.h"
#include "../common/timing.h"

namespace a2d2_ros_preparer {

    class CameraImageTimeseriesPerView {
    public:
        explicit CameraImageTimeseriesPerView(const std::filesystem::path& directory_path,
                                              const std::string& frame_id,
                                              const std::map<DataSequenceId, Time>& data_sequence_time_mins,
                                              const std::map<DataSequenceId, Time>& data_sequence_time_maxs,
                                              const Duration& time_offset,
                                              const std::string& field_name_image_time,
                                              const double& time_estimation_lidar_timeframe_to_image_ratio,
                                              const std::optional<Duration>& time_estimation_time_tolerance);

        sensor_msgs::ImagePtr GetImage(DataSequenceId sequence_id);

    private:
        std::string frame_id_;
        std::map<DataSequenceId, std::filesystem::path> image_filepaths_;
        std::map<DataSequenceId, Time> image_times_;
    };
}



#endif //A2D2_ROS_PREPARER_CAMERA_IMAGE_TIMESERIES_PER_VIEW_H
