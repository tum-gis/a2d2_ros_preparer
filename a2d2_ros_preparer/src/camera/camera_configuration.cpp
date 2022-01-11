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

    sensor_msgs::CameraInfo CameraConfiguration::GetCameraInfoMessage(std::string& frame_id, ros::Time& stamp) const {
        auto header = std_msgs::Header();
        header.frame_id = frame_id;
        header.stamp = stamp;

        auto message = sensor_msgs::CameraInfo();
        message.header = header;
        message.width = width_;
        message.height = height_;

        message.distortion_model = "plumb_bob";
        message.D = GetDistortion();
        message.K = ToBoostArray9d(camera_matrix_original_);
        Eigen::Matrix<double, 3, 4> projection_camera_matrix = Eigen::MatrixXd::Zero(3, 4);
        projection_camera_matrix.block<3,3>(0,0) = camera_matrix_;
        message.P = ToBoostArray12d(projection_camera_matrix);

        return message;
    }

    Eigen::VectorXd CameraConfiguration::GetDistortionEigen() const {
        auto distortion = GetDistortion();
        return Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(distortion.data(), static_cast<int>(distortion.size()));
    }
}
