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

#ifndef A2D2_ROS_PREPARER_ARGUMENT_PARSER_H
#define A2D2_ROS_PREPARER_ARGUMENT_PARSER_H

#include "options.h"

namespace a2d2_ros_preparer {

    template<typename T>
    T GetParam(const ros::NodeHandle& node_handle, const std::string& key)
    {
        CHECK(node_handle.hasParam(key)) << "Value for " << key << " must be available.";

        T result;
        node_handle.getParam(key, result);
        return result;
    }

    class OptionsParser {
    public:

        static Options Parse();

    private:
        static void ParsePaths(const ros::NodeHandle& node, const std::string& prefix, Options& options);
        static void ParseFilter(const ros::NodeHandle& node, const std::string& prefix, Options& options);
        static void ParseLidarSensors(const ros::NodeHandle& node, const std::string& prefix, Options& options);
        static void ParseCameraSensors(const ros::NodeHandle& node, const std::string& prefix, Options& options);
        static void ParseBusSignals(const ros::NodeHandle& node, std::string prefix, Options& options);
        static void ParsePublish(const ros::NodeHandle& node, const std::string& prefix, Options& options);
        static void ParseWrites(const ros::NodeHandle& node, const std::string& prefix, Options& options);
    };

}

#endif //A2D2_ROS_PREPARER_ARGUMENT_PARSER_H
