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


#ifndef A2D2_ROS_PREPARER_VECTOR_CONVERSION_H
#define A2D2_ROS_PREPARER_VECTOR_CONVERSION_H

#include <eigen_conversions/eigen_msg.h>
#include <nlohmann/json.hpp>

namespace a2d2_ros_preparer {

    Eigen::Vector3d ToEigenVector3d(const nlohmann::json& vector);
    Eigen::Matrix3d ToEigenMatrix3d(const nlohmann::json& matrix);

    boost::array<double, 9> ToBoostArray9d(const Eigen::Matrix3d& matrix);
    boost::array<double, 12> ToBoostArray12d(const Eigen::MatrixXd& matrix);

    Eigen::Affine3d GetTransformFromTo(const nlohmann::json& source_view, const nlohmann::json& target_view, double epsilon);
    Eigen::Matrix4d GetTransformToGlobal(const nlohmann::json& view, double epsilon);
    Eigen::Matrix4d GetTransformFromGlobal(const nlohmann::json& view, double epsilon);

    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> GetAxes(const nlohmann::json& view, double epsilon);
    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> GetAxes(const Eigen::Vector3d& x_axis, const Eigen::Vector3d& y_axis, double epsilon);

}


#endif //A2D2_ROS_PREPARER_VECTOR_CONVERSION_H
