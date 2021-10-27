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

#include <ros/ros.h>

#include "options_parser.h"
#include "converter.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "converter");

    auto options = a2d2_ros_preparer::OptionsParser::Parse();
    options.ValidateAndPrepare();

    a2d2_ros_preparer::Converter::Process(options);

    return 0;
}
