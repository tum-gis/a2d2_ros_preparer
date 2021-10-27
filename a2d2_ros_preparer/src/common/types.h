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


#ifndef A2D2_ROS_PREPARER_TYPES_H
#define A2D2_ROS_PREPARER_TYPES_H

#include <cmath>

namespace a2d2_ros_preparer {

    using DataSequenceId = unsigned int;
    using CameraDirectionIdentifier = std::string;
    using LidarDirectionIdentifier = std::string;

    inline constexpr double DegToRad(double deg) { return M_PI * deg / 180.0; }
    inline constexpr double KmhToMps(double kmh) { return 5.0 * kmh / 18.0; }
    inline constexpr double MpsToKmh(double mps) { return 18.0 * mps / 5.0; }
    inline constexpr uint64_t MillisecondsToNanoseconds(uint64_t milliseconds) { return milliseconds * 1000; }

}


#endif //A2D2_ROS_PREPARER_TYPES_H
