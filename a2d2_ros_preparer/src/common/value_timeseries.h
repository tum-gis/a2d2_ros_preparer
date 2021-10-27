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

#ifndef A2D2_ROS_PREPARER_VALUE_TIMESERIES_H
#define A2D2_ROS_PREPARER_VALUE_TIMESERIES_H

#include <optional>
#include <deque>
#include <nlohmann/json.hpp>
#include <glog/logging.h>
#include "timing.h"

namespace a2d2_ros_preparer {

    template <typename V>
    class ValueTimeseries {
    public:
        [[nodiscard]] size_t size() const { return entries_.size(); };
        [[nodiscard]] bool empty() const { return entries_.empty(); };
        void push_back(const std::pair<Time, V> &entry) {
            if (!empty())
                CHECK_GT(entry.first, back().first) << "Must be strictly ordered according to time";

            entries_.push_back(entry);
        }

        [[nodiscard]] size_t GetLowerIndexForTime(Time time) const {
            if (empty()) {
                return -1;
            }

            auto lower = std::lower_bound(entries_.begin(), entries_.end(), std::pair<Time, V>(time, 0),
                                          [](const std::pair<Time, V>& a, const std::pair<Time, V>& b) { return a.first < b.first; });
            auto index = std::distance(entries_.begin(), lower);
            return index;
        }

        V GetValue(Time time) {
            if (empty() || time < entries_.front().first || entries_.back().first < time) {
                CHECK(default_value_.has_value()) << "No default value set";
                return default_value_.value();
            }

            auto index = GetLowerIndexForTime(time);
            return entries_.at(index).second;
        }

        Time GetStartTime() {
            if (default_value_.has_value())
                return TimeMin;
            return front().first;
        };
        Time GetStopTime() {
            if (default_value_.has_value())
                return TimeMax;
            return back().first;
        };

        void SetDefaultValue(V value) { default_value_ = value; };

        const std::pair<Time, V>& front() const { return entries_.front(); };
        const std::pair<Time, V>& back() const { return entries_.back(); };

    private:
        std::deque<std::pair<Time, V>> entries_;
        std::optional<V> default_value_;
    };

    template <typename V>
    ValueTimeseries<V> LoadFromJSON(const nlohmann::json& json_array, Duration time_offset, V value_offset, V value_factor) {
        auto timeseries = ValueTimeseries<V>();
        for (auto it : json_array)
        {
            Time time = 0;
            if (it[0].is_number_integer()) {
                // interpret timestamp as milliseconds
                time = MillisecondsToNanoseconds(it[0]);
            }
            else if (it[0].is_number_float()) {
                // interpret timestamp as seconds
                time = FromUniversalSeconds(it[0]);
            }

            V value = it[1];
            auto entry = std::make_pair(time + time_offset, (value * value_factor) + value_offset);
            timeseries.push_back(entry);
        }

        return timeseries;
    }

}

#endif //A2D2_ROS_PREPARER_VALUE_TIMESERIES_H
