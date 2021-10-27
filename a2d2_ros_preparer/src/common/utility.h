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


#ifndef A2D2_ROS_PREPARER_UTILITY_H
#define A2D2_ROS_PREPARER_UTILITY_H

#include <regex>
#include <array>
#include <filesystem>

namespace a2d2_ros_preparer {

    template <typename T>
    bool ContainsDeviationGreaterThanThreshold(const std::vector<T>& values, const T& main, T threshold) {
        for (auto const& current_value: values) {
            if (std::abs(current_value-main) > threshold)
                return true;
        }
        return false;
    }

    template <typename T>
    T GetMedian(std::vector<T>& values)
    {
        size_t n = values.size() / 2;
        std::nth_element(values.begin(), values.begin()+n, values.end());
        return values.at(n);
    }

    unsigned int ExtractLastIntegerFromString(const std::string& s);
    int64_t CalculateMean(const std::vector<int64_t>& values);

    bool IsSubdirectoryContained(const std::filesystem::path& base_directory, const std::string& directory_name_substring);
    std::filesystem::path GetSubdirectoryPath(const std::filesystem::path& base_directory, const std::string& directory_name_substring);
}

#endif //A2D2_ROS_PREPARER_UTILITY_H
