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


#ifndef A2D2_ROS_PREPARER_CAMERA_CONFIGURATION_H
#define A2D2_ROS_PREPARER_CAMERA_CONFIGURATION_H

#include <sensor_msgs/CameraInfo.h>
#include <eigen_conversions/eigen_msg.h>

namespace a2d2_ros_preparer {

    enum class CameraLensType {
        TELECAM,
        FISHEYE
    };

    class CameraConfiguration {

    public:
        CameraConfiguration(uint32_t width, uint32_t height,
                            CameraLensType lens_type,
                            Eigen::Matrix3d& camera_matrix,
                            Eigen::Matrix3d& camera_matrix_original,
                            std::vector<double>& distortion):
                            width_(width),
                            height_(height),
                            lens_type_(lens_type),
                            camera_matrix_(camera_matrix),
                            camera_matrix_original_(camera_matrix_original) {};

        [[nodiscard]] sensor_msgs::CameraInfo GetCameraInfoMessage(std::string& frame_id, ros::Time& stamp) const;

        [[nodiscard]] CameraLensType GetLensType() const { return lens_type_; };
        [[nodiscard]] Eigen::Matrix3d GetCameraMatrix() const { return camera_matrix_; };
        [[nodiscard]] Eigen::Matrix3d GetCameraMatrixOriginal() const { return camera_matrix_original_; };
        [[nodiscard]] std::vector<double> GetDistortion() const { return distortion_; };
        [[nodiscard]] Eigen::VectorXd GetDistortionEigen() const;

    private:
        uint32_t width_;
        uint32_t height_;

        CameraLensType lens_type_;

        Eigen::Matrix3d camera_matrix_;
        Eigen::Matrix3d camera_matrix_original_;
        std::vector<double> distortion_;
    };

}

#endif //A2D2_ROS_PREPARER_CAMERA_CONFIGURATION_H
