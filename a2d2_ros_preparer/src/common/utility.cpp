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

#include "utility.h"

#include <glog/logging.h>

namespace a2d2_ros_preparer {

    unsigned int ExtractLastIntegerFromString(const std::string& s) {
        std::regex rgx("(\\d+)(?!.*\\d)");
        std::smatch match;
        if (std::regex_search(s.begin(), s.end(), match, rgx))
            return stoi(match[1]);
        else
            throw std::runtime_error(std::string("No integer found in string."));
    }

    int64_t CalculateMean(const std::vector<int64_t> &values) {
        int64_t average = 0;
        int t = 1;
        for (const int64_t current_value: values) {
            average += (current_value - average) / t;
            ++t;
        }
        return average;
    }

    bool IsSubdirectoryContained(const std::filesystem::path& base_directory, const std::string& directory_name_substring) {
        CHECK(exists(base_directory)) << "No base directory found at " << base_directory;

        for(const auto& current_directory : std::filesystem::directory_iterator(base_directory)) {
            auto current_directory_name = current_directory.path().filename().string();
            if (current_directory_name.find(directory_name_substring) != std::string::npos)
                return true;
        }

        return false;
    }

    std::filesystem::path GetSubdirectoryPath(const std::filesystem::path& base_directory, const std::string& directory_name_substring) {
        CHECK(exists(base_directory)) << "No base directory found at " << base_directory;

        std::vector<std::filesystem::path> filepaths;

        for (const auto & current_directory : std::filesystem::directory_iterator(base_directory)) {
            auto current_directory_name = current_directory.path().filename().string();
            if (current_directory_name.find(directory_name_substring) != std::string::npos)
                filepaths.push_back(current_directory);
        }

        CHECK_LE(filepaths.size(), 1) << "Multiple subdirectories found for substring: " << directory_name_substring;
        CHECK_NE(filepaths.size(), 0) << "No subdirectory found for substring: " << directory_name_substring;
        return filepaths.front();
    }

}
