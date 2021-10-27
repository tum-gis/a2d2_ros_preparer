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


#include <glog/logging.h>
#include "vector_conversion.h"

namespace a2d2_ros_preparer {

    Eigen::Vector3d ToEigenVector3d(const nlohmann::json& vector) {
        CHECK_EQ(vector.size(), 3) << "JSON must contain exactly three elements for creating a vector in 3D.";

        double x = vector[0];
        double y = vector[1];
        double z = vector[2];

        auto eigen_vector = Eigen::Vector3d(x, y, z);
        CHECK(eigen_vector.allFinite()) << "Values of vector must all be finite.";
        return eigen_vector;
    }

    Eigen::Matrix3d ToEigenMatrix3d(const nlohmann::json& matrix) {
        CHECK_EQ(matrix.size(), 3) << "JSON must contain exactly three elements for creating a 3x3 matrix";

        auto eigen_matrix = Eigen::Matrix3d();

        for (int current_row_index = 0; current_row_index < 3; ++current_row_index) {
            for (int current_column_index = 0; current_column_index < 3; ++current_column_index) {
                eigen_matrix(current_row_index, current_column_index) = matrix[current_row_index][current_column_index];
            }
        }

        CHECK(eigen_matrix.allFinite()) << "Values of matrix must all be finite.";
        return eigen_matrix;
    }

    boost::array<double, 9> ToBoostArray9d(const Eigen::Matrix3d& matrix) {
        CHECK_EQ(matrix.size(), 9) << "Matrix must contain 9 elements.";

        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> matrix_row_major;
        matrix_row_major = matrix;

        auto boost_array = boost::array<double, 9>();
        for (int i = 0; i < matrix_row_major.size(); i++)
            boost_array[i] = *(matrix_row_major.data() + i);

        return boost_array;
    }

    boost::array<double, 12> ToBoostArray12d(const Eigen::MatrixXd& matrix) {
        CHECK_EQ(matrix.size(), 12) << "Matrix must contain 12 elements.";

        Eigen::Matrix<double, 3, 4, Eigen::RowMajor> matrix_row_major;
        matrix_row_major = matrix;

        auto boost_array = boost::array<double, 12>();
        for (int i = 0; i < matrix_row_major.size(); i++)
            boost_array[i] = *(matrix_row_major.data() + i);

        return boost_array;
    }

    Eigen::Affine3d GetTransformFromTo(const nlohmann::json& source_view, const nlohmann::json& target_view, double epsilon) {
        auto from = GetTransformFromGlobal(target_view, epsilon);
        auto to = GetTransformToGlobal(source_view, epsilon);
        auto transformation_matrix = from * to;

        Eigen::Affine3d affine;
        affine.matrix() = transformation_matrix;
        return affine;
    }

    Eigen::Matrix4d GetTransformFromGlobal(const nlohmann::json& view, double epsilon) {
        auto transform_to_global = GetTransformToGlobal(view, epsilon);

        Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();

        Eigen::Matrix3d rot = transform_to_global.block<3,3>(0,0).transpose();
        trans.block<3,3>(0,0) = rot;
        trans.block<3,1>(0, 3) = rot * (-transform_to_global.block<3,1>(0, 3));

        return trans;
    }

    Eigen::Matrix4d GetTransformToGlobal(const nlohmann::json& view, double epsilon) {
        auto axes = GetAxes(view, epsilon);
        auto x_axis = std::get<0>(axes);
        auto y_axis = std::get<1>(axes);
        auto z_axis = std::get<2>(axes);

        // get origin
        auto origin = ToEigenVector3d(view["origin"]);
        Eigen::Matrix4d transform_to_vehicle = Eigen::Matrix4d::Identity();

        // rotation
        transform_to_vehicle.block<3,1>(0,0) = x_axis;
        transform_to_vehicle.block<3,1>(0,1) = y_axis;
        transform_to_vehicle.block<3,1>(0,2) = z_axis;

        // origin
        transform_to_vehicle.block<3,1>(0,3) = origin;

        return transform_to_vehicle;
    }

    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> GetAxes(const nlohmann::json& view, double epsilon) {
        auto x_axis = ToEigenVector3d(view["x-axis"]);
        auto y_axis = ToEigenVector3d(view["y-axis"]);
        return GetAxes(x_axis, y_axis, epsilon);
    }

    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> GetAxes(const Eigen::Vector3d& x_axis, const Eigen::Vector3d& y_axis, double epsilon) {
        CHECK_GT(x_axis.norm(), epsilon) << "Norm of x_axis is too small.";
        CHECK_GT(y_axis.norm(), epsilon) << "Norm of y_axis is too small.";

        // normalize the axes
        const auto x_axis_normalized = x_axis.normalized();
        const auto y_axis_normalized = y_axis.normalized();

        // make a new y-axis which lies in the original x-y plane, but is orthogonal to x-axis
        const auto y_axis_orthogonal = y_axis_normalized - x_axis_normalized * y_axis_normalized.dot(x_axis_normalized);

        // create orthogonal z-axis
        const auto z_axis = x_axis_normalized.cross(y_axis_normalized);

        // calculate and check y-axis and z-axis norms
        CHECK_GT(y_axis_orthogonal.norm(), epsilon) << "Norm of y_axis_orthogonal is too small.";
        CHECK_GT(z_axis.norm(), epsilon) << "Norm of z_axis is too small.";

        // make x/y/z-axes orthonormal
        return std::make_tuple(x_axis_normalized, y_axis_orthogonal.normalized(), z_axis.normalized());
    }
}
