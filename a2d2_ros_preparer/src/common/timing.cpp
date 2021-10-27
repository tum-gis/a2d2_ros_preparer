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

#include "timing.h"

namespace a2d2_ros_preparer {

    Time FromUniversalSeconds(double seconds) {
        return static_cast<uint64_t>(seconds*1e9);
    }

    Duration FromSeconds(double seconds) {
        return static_cast<Duration>(seconds*1e9);
    }

    double_t ToSeconds(Duration duration) {
        return static_cast<double_t>(duration)*1e-9;
    }

    double_t ToUniversalSeconds(Time time) {
        return static_cast<double_t>(time)*1e-9;
    }

    ros::Time ToRosTime(Time time) {
        uint32_t seconds = time * 1e-9;
        uint32_t nano_seconds = time - seconds * 1e9;
        return ros::Time(seconds, nano_seconds);
    }
}
