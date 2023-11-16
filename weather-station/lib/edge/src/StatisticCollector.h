/*
 * Copyright (c) 2023 Particle Industries, Inc.
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

#pragma once

/**
 * @brief A class that collects statistics about a set of time series values and keeps track of minimum, maximum, and exponential running averages.
 *
 * @tparam T The data type of the values to be collected. Must support the arithmetic operators + and *.
 */
template<typename T>
class StatisticCollector {
public:
    /**
     * @brief Constructs a new StatisticCollector object.
     *
     * @param alpha The alpha value to use for calculating the running average. Defaults to 1.0.
     * @param minMaxAfterAverage If true, the minimum and maximum values are updated after computing the running average. Defaults to false.
     */
    StatisticCollector(T alpha = 1.0, bool minMaxAfterAverage = false) :
        _alpha(alpha),
        _oneMinusAlpha(1.0 - alpha),
        _minMaxAfterAverage(minMaxAfterAverage) {}

    /**
     * @brief Gets the minimum value collected by the StatisticCollector object.
     *
     * @return The minimum value.
     */
    T getMin() const { return _min; }

    /**
     * @brief Gets the maximum value collected by the StatisticCollector object.
     *
     * @return The maximum value.
     */
    T getMax() const { return _max; }

    /**
     * @brief Gets the running average of the values collected by the StatisticCollector object.
     *
     * @return The running average.
     */
    T getAverage() const { return _runningAvg; }

    /**
     * @brief Gets the alpha value used for calculating the exponential running average.
     *
     * @return The alpha value.
     */
    T getAverageAlpha() const { return _alpha; }

    /**
     * @brief Sets the alpha value used for calculating the exponential running average.
     *
     * @param alpha The new alpha value.
     * @return 0 on success, -1 if the alpha value is outside the valid range.
     */
    int setAverageAlpha(T alpha)
    {
        if ((0.0 > alpha) || (1.0 < alpha)) {
            return -1;
        }
        _alpha = alpha;
        _oneMinusAlpha = 1.0 - alpha;
        return 0;
    }

    /**
     * @brief Clears the collected statistics.
     */
    void clear() {
        _min = {};
        _max = {};
        _runningAvg = {};
        _first = true;
    }

    /**
     * @brief Adds a new value to the collected statistics.
     *
     * @param value The value to add.
     */
    void pushValue(T value) {
        if (__builtin_expect(!!(_first), 0)) {
            _min = _max = _runningAvg = value;
            _first = false;
        } else {
            _runningAvg = (value * _alpha) + (_runningAvg * _oneMinusAlpha);

            if (_minMaxAfterAverage) {
                value = _runningAvg;
            }
            _min = min(_min, value);
            _max = max(_max, value);
        }
    }

    /**
     * @brief Helper function to convert a cutoff frequency for low-pass filtering to an alpha value.
     *
     * @param dt The sampling rate in seconds.
     * @param fc The cutoff frequency in Hertz.
     * @return The calculated alpha coefficient.
     */
    static inline T frequencyToAlpha(T dt, T fc) {
        // See https://en.wikipedia.org/wiki/Exponential_smoothing#Time_constant
        return 1.0 - std::exp(-dt * 2.0 * M_PI * fc);
    }

private:
    bool _first {true};             ///< Flag indicating if the first value has been added.
    float _min {};                  ///< The minimum value.
    float _max {};                  ///< The maximum value.
    float _runningAvg {};           ///< The exponential running average.
    float _alpha {1.0F};            ///< The alpha value used for calculating the exponential running average.
    float _oneMinusAlpha {0.0F};    ///< 1.0 - _alpha, cached for performance.
    bool _minMaxAfterAverage {};    ///< Flag indicating if the minimum and maximum values are updated after computing the running average.
};
