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


#ifndef A2D2_ROS_PREPARER_NPZ_FILE_READER_H
#define A2D2_ROS_PREPARER_NPZ_FILE_READER_H

#include <filesystem>
#include "timed_point_cloud_data.h"

namespace a2d2_ros_preparer {

    TimedPointCloudData ReadFile(const std::filesystem::path& , const std::string& view_id, const DataSequenceId& sequence_id);

}

#endif //A2D2_ROS_PREPARER_NPZ_FILE_READER_H
