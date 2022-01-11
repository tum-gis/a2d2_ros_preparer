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


#include "camera_image_stream.h"

#include <utility>
#include <glog/logging.h>
#include "../common/utility.h"

namespace a2d2_ros_preparer {

    CameraImageStream::CameraImageStream(const std::filesystem::path& camera_data_directory,
                                         const std::vector<CameraDirectionIdentifier>& camera_identifiers,
                                         const std::set<CameraDirectionIdentifier>& filter_camera_identifiers,
                                         std::map<CameraDirectionIdentifier, CameraConfiguration> camera_configurations,
                                         const std::map<CameraDirectionIdentifier, std::map<DataSequenceId, Time>>& data_sequence_time_mins_per_view,
                                         const std::map<CameraDirectionIdentifier, std::map<DataSequenceId, Time>>& data_sequence_time_maxs_per_view,
                                         const Duration& time_offset,
                                         const std::string& field_name_image_time,
                                         const double& time_estimation_lidar_timeframe_to_image_ratio,
                                         const std::optional<Duration>& time_estimation_time_tolerance) {

        if (!exists(camera_data_directory)) {
            LOG(WARNING) << "No camera data directory found at " << camera_data_directory;
            return;
        }

        for (const auto& current_camera_identifier: camera_identifiers) {
            if (!filter_camera_identifiers.empty() && filter_camera_identifiers.count(current_camera_identifier) == 0)
                continue;
            if (!IsSubdirectoryContained(camera_data_directory, current_camera_identifier)) {
                LOG(WARNING) << "Camera data directory for direction '" << current_camera_identifier << "' not found.";
                continue;
            }

            const auto& camera_configuration = camera_configurations.at(current_camera_identifier);
            const auto& data_sequence_time_mins = data_sequence_time_mins_per_view.at(current_camera_identifier);
            const auto& data_sequence_time_maxs = data_sequence_time_maxs_per_view.at(current_camera_identifier);

            auto directory_path = GetSubdirectoryPath(camera_data_directory, current_camera_identifier);
            auto timeseries = CameraImageTimeseriesPerView(directory_path,
                                                           current_camera_identifier,
                                                           data_sequence_time_mins,
                                                           data_sequence_time_maxs,
                                                           time_offset,
                                                           field_name_image_time,
                                                           time_estimation_lidar_timeframe_to_image_ratio,
                                                           time_estimation_time_tolerance,
                                                           camera_configuration);

            camera_image_timeseries_per_view_.insert(std::make_pair(current_camera_identifier, timeseries));
        }
    }

    std::vector<CameraDirectionIdentifier> CameraImageStream::GetCameraIdentifiers() const {
        std::vector<CameraDirectionIdentifier> camera_identifiers;
        for(const auto& [key, _] : camera_image_timeseries_per_view_)
            camera_identifiers.push_back(key);
        return camera_identifiers;
    }

    void CameraImageStream::WriteDistortedCameraDataToRosbag(rosbag::Bag &bag, const CameraDirectionIdentifier& camera_identifier,
                                                    const std::vector<DataSequenceId>& sequence_ids) {
        if (camera_image_timeseries_per_view_.count(camera_identifier) == 0)
            return;

        auto camera_image_timeseries = camera_image_timeseries_per_view_.at(camera_identifier);
        for (auto current_sequence_id: sequence_ids) {
            auto image_message = camera_image_timeseries.GetOriginalImageMessage(current_sequence_id);
            bag.write("/camera_" + camera_identifier + "/rgb/image_color", image_message->header.stamp, image_message);
        }
    }

    void CameraImageStream::WriteRectifiedCameraDataToRosbag(rosbag::Bag &bag, const CameraDirectionIdentifier& camera_identifier,
                                                             const std::vector<DataSequenceId>& sequence_ids) {
        if (camera_image_timeseries_per_view_.count(camera_identifier) == 0)
            return;

        auto camera_image_timeseries = camera_image_timeseries_per_view_.at(camera_identifier);
        for (auto current_sequence_id: sequence_ids) {
            auto image_message = camera_image_timeseries.GetRectifiedImageMessage(current_sequence_id);
            bag.write("/camera_" + camera_identifier + "/rgb/image_rect_color", image_message->header.stamp, image_message);
        }
    }

    void CameraImageStream::WriteCameraInfoDataToRosbag(rosbag::Bag &bag, const CameraDirectionIdentifier& camera_identifier,
                                                             const std::vector<DataSequenceId>& sequence_ids) {
        if (camera_image_timeseries_per_view_.count(camera_identifier) == 0)
            return;

        auto camera_image_timeseries = camera_image_timeseries_per_view_.at(camera_identifier);
        for (auto current_sequence_id: sequence_ids) {
            auto camera_info_message = camera_image_timeseries.GetCameraInfoMessage(current_sequence_id);
            bag.write("/camera_" + camera_identifier + "/rgb/camera_info", camera_info_message.header.stamp, camera_info_message);
        }
    }

}
