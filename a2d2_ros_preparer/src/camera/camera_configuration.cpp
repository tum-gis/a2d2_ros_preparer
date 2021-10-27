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


#include "camera_configuration.h"

#include <utility>
#include "../common/vector_conversion.h"

namespace a2d2_ros_preparer {

    CameraConfiguration::CameraConfiguration(uint32_t width, uint32_t height,
                                             std::vector<double> distortion,
                                             Eigen::Matrix3d  intrinsic_camera_matrix,
                                             Eigen::MatrixXd  projection_camera_matrix):
                                             width_(width), height_(height),
                                             distortion_(std::move(distortion)),
                                             intrinsic_camera_matrix_(std::move(intrinsic_camera_matrix)),
                                             projection_camera_matrix_(std::move(projection_camera_matrix)) {};

    sensor_msgs::CameraInfo CameraConfiguration::GetRosMessage() {
        auto message = sensor_msgs::CameraInfo();

        message.width = width_;
        message.height = height_;

        message.D = distortion_;
        message.K = ToBoostArray9d(intrinsic_camera_matrix_);
        message.P = ToBoostArray12d(projection_camera_matrix_);

        return message;
    }
}