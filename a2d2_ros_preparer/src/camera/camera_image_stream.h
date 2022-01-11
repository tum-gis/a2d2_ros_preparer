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


#ifndef A2D2_ROS_PREPARER_CAMERA_IMAGE_STREAM_H
#define A2D2_ROS_PREPARER_CAMERA_IMAGE_STREAM_H

#include <map>
#include <vector>
#include <rosbag/bag.h>
#include <nlohmann/json.hpp>
#include "../common/timing.h"
#include "camera_image_timeseries_per_view.h"
#include "camera_configuration.h"

namespace a2d2_ros_preparer {

    class CameraImageStream {

    public:
        CameraImageStream(const std::filesystem::path& camera_data_directory,
                          const std::vector<CameraDirectionIdentifier>& camera_identifiers,
                          const std::set<CameraDirectionIdentifier>& filter_camera_identifiers,
                          std::map<CameraDirectionIdentifier, CameraConfiguration> camera_configurations,
                          const std::map<CameraDirectionIdentifier, std::map<DataSequenceId, Time>>& data_sequence_time_mins_per_view,
                          const std::map<CameraDirectionIdentifier, std::map<DataSequenceId, Time>>& data_sequence_time_maxs_per_view,
                          const Duration& time_offset,
                          const std::string& field_name_image_time,
                          const double& time_estimation_lidar_timeframe_to_image_ratio,
                          const std::optional<Duration>& time_estimation_time_tolerance);

        void WriteDistortedCameraDataToRosbag(rosbag::Bag &bag, const CameraDirectionIdentifier& camera_identifier, const std::vector<DataSequenceId>& sequence_ids);
        void WriteRectifiedCameraDataToRosbag(rosbag::Bag &bag, const CameraDirectionIdentifier& camera_identifier, const std::vector<DataSequenceId>& sequence_ids);

        void WriteCameraInfoDataToRosbag(rosbag::Bag &bag, const CameraDirectionIdentifier& camera_identifier, const std::vector<DataSequenceId>& sequence_ids);

        [[nodiscard]] std::vector<CameraDirectionIdentifier> GetCameraIdentifiers() const;

    private:
        std::map<CameraDirectionIdentifier, CameraImageTimeseriesPerView> camera_image_timeseries_per_view_;
    };
}

#endif //A2D2_ROS_PREPARER_CAMERA_IMAGE_STREAM_H
