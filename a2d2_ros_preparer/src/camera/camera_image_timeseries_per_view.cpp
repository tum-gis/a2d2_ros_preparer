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

#include "camera_image_timeseries_per_view.h"

#include <fstream>
#include <nlohmann/json.hpp>
#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <utility>
#include "../common/utility.h"

namespace a2d2_ros_preparer {

    CameraImageTimeseriesPerView::CameraImageTimeseriesPerView(const std::filesystem::path& directory_path,
                                                               const std::string& frame_id,
                                                               const std::map<DataSequenceId, Time>& data_sequence_time_mins,
                                                               const std::map<DataSequenceId, Time>& data_sequence_time_maxs,
                                                               const Duration& time_offset,
                                                               const std::string& field_name_image_time,
                                                               const double& image_time_estimation_ratio,
                                                               const std::optional<Duration>& time_estimation_time_tolerance,
                                                               CameraConfiguration  camera_configuration):
                                                               frame_id_(frame_id),
                                                               camera_configuration_(std::move(camera_configuration)) {
        CHECK_EQ(data_sequence_time_mins.size(), data_sequence_time_maxs.size()) << "Time mins and time maxs of data sequence must have the same size";
        CHECK(std::equal(data_sequence_time_mins.begin(), data_sequence_time_mins.end(), data_sequence_time_maxs.begin(),
                         [] (auto a, auto b) { return a.first == b.first; })) << "Time mins and time maxs must be available for the same sequence ids";

        LOG(INFO) << "Reading camera data for view: " << frame_id;

        std::map<DataSequenceId, std::filesystem::path> image_filepaths;
        std::map<DataSequenceId, std::filesystem::path> meta_information_filepaths;
        for (const auto& entry : std::filesystem::directory_iterator(directory_path)) {
            if (entry.path().extension() == ".png") {
                const std::string stem = entry.path().stem();
                DataSequenceId id = ExtractLastIntegerFromString(stem);
                image_filepaths[id] = entry.path();
            } else if (entry.path().extension() == ".json") {
                const std::string stem = entry.path().stem();
                DataSequenceId id = ExtractLastIntegerFromString(stem);
                meta_information_filepaths[id] = entry.path();
            }
        }

        // assign relevant file paths for images
        std::map<DataSequenceId, Time> meta_information_times;
        for (const auto& [current_data_sequence_id, _]: data_sequence_time_mins) {
            image_filepaths_[current_data_sequence_id] = image_filepaths.at(current_data_sequence_id);

            if (meta_information_filepaths.count(current_data_sequence_id) > 0) {
                std::ifstream jsonFile(meta_information_filepaths.at(current_data_sequence_id));
                nlohmann::json meta_informatiom_json;
                jsonFile >> meta_informatiom_json;

                if (meta_informatiom_json.contains(field_name_image_time)) {
                    uint64_t extracted_time_ms = meta_informatiom_json[field_name_image_time];
                    meta_information_times[current_data_sequence_id] = extracted_time_ms*1000 + time_offset;
                }

            }
        }

        bool all_meta_information_available = std::equal(image_filepaths_.begin(), image_filepaths_.end(),
                                                         meta_information_times.begin(),
                                                         [] (auto a, auto b) { return a.first == b.first; });
        if (all_meta_information_available) {
            for (const auto& [current_data_sequence_id, _]: image_filepaths_)
                image_times_[current_data_sequence_id] = meta_information_times[current_data_sequence_id];
        }

        if (!all_meta_information_available || time_estimation_time_tolerance.has_value()) {
            Time start_time = data_sequence_time_mins.begin()->second +
                    static_cast<uint64_t>(image_time_estimation_ratio *
                    static_cast<double>(data_sequence_time_maxs.begin()->second - data_sequence_time_mins.begin()->second));
            Time end_time = data_sequence_time_mins.rbegin()->second +
                    static_cast<uint64_t>(image_time_estimation_ratio *
                    static_cast<double>(data_sequence_time_maxs.rbegin()->second - data_sequence_time_mins.rbegin()->second));
            auto period_duration = (end_time - start_time) / (data_sequence_time_mins.size() - 1);

            std::map<DataSequenceId, Time>  estimated_image_times = {};
            for (const auto& [current_data_sequence_id, current_data_sequence_time_min]: data_sequence_time_mins) {
                auto current_counter = current_data_sequence_id - data_sequence_time_mins.begin()->first;
                auto current_image_time = start_time + period_duration * current_counter;
                estimated_image_times[current_data_sequence_id] = current_image_time;
            }

            if (!all_meta_information_available) {
                LOG(INFO) << "Estimating image times for view " << frame_id;
                image_times_ = estimated_image_times;
            }

            if (time_estimation_time_tolerance.has_value()) {
                int deviation_counter = 0;
                for (const auto& [current_data_sequence_id, current_image_time]: estimated_image_times) {
                    if (meta_information_times.count(current_data_sequence_id) > 0) {
                        auto time_difference = static_cast<int64_t>(current_image_time - meta_information_times.at(current_data_sequence_id));
                        if (std::abs(time_difference) > time_estimation_time_tolerance.value())
                            deviation_counter++;
                    }

                }
                if (deviation_counter > 0)
                    LOG(WARNING) << "Identified " << deviation_counter << "/" << meta_information_times.size() <<
                    " deviations (tolerance: " << ToSeconds(time_estimation_time_tolerance.value()) <<
                    "s) between provided meta information and time estimation method for view '" << frame_id << "'";
            }
        }
    }

    sensor_msgs::ImagePtr CameraImageTimeseriesPerView::GetOriginalImageMessage(DataSequenceId sequence_id) {
        auto header = GetPreparedImageMessageHeader(sequence_id);

        auto filepath = image_filepaths_.at(sequence_id);
        cv::Mat src_image = cv::imread(filepath);

        return cv_bridge::CvImage(header, "bgr8", src_image).toImageMsg();
    }

    sensor_msgs::ImagePtr CameraImageTimeseriesPerView::GetRectifiedImageMessage(DataSequenceId sequence_id) {
        auto header = GetPreparedImageMessageHeader(sequence_id);

        auto filepath = image_filepaths_.at(sequence_id);
        cv::Mat src_image = cv::imread(filepath);
        cv::Mat undistorted_image;

        cv::Mat cam_matrix_original_cv(3, 3, CV_64F);
        cv::eigen2cv(camera_configuration_.GetCameraMatrixOriginal(), cam_matrix_original_cv);
        cv::Mat cam_matrix_cv(3, 3, CV_64F);
        cv::eigen2cv(camera_configuration_.GetCameraMatrix(), cam_matrix_cv);
        cv::Mat distortion(static_cast<int>(camera_configuration_.GetDistortion().size()), 1, CV_64F);
        cv::eigen2cv(camera_configuration_.GetDistortionEigen(), distortion);

        if (camera_configuration_.GetLensType() == CameraLensType::TELECAM)
            cv::undistort(src_image, undistorted_image, cam_matrix_original_cv, distortion, cam_matrix_cv);
        else
            cv::fisheye::undistortImage(src_image, undistorted_image, cam_matrix_original_cv, distortion, cam_matrix_cv);

        return cv_bridge::CvImage(header, "bgr8", undistorted_image).toImageMsg();
    }

    sensor_msgs::CameraInfo CameraImageTimeseriesPerView::GetCameraInfoMessage(DataSequenceId sequence_id) {
        std::string frame_id = "camera_" + frame_id_;
        ros::Time stamp = ToRosTime(image_times_.at(sequence_id));

        return camera_configuration_.GetCameraInfoMessage(frame_id, stamp);
    }

    std_msgs::Header CameraImageTimeseriesPerView::GetPreparedImageMessageHeader(DataSequenceId sequence_id) {
        auto header = std_msgs::Header();
        header.frame_id = "camera_" + frame_id_;
        header.stamp = ToRosTime(image_times_.at(sequence_id));
        return header;
    }
}
