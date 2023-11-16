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

#include <functional>
#include <type_traits>

// The ThresholdComparator evaluates a given value against a user specified threshold and hysteresis.
//
//                              Above
//              +---------------------
//              |     :     ^
//              |     :     |
//              |     :     |
//    Below     v     :     |
//   -----------------------+
//                    :
//               <--->:<--->
//               hyst : hyst
//                    :
//                threshold
//
// If prevously AboveThreshold, the success of less than (or equal) threshold - hysteresis will transition
// the current state to BelowThreshold.
//
// If previously BelowThreshold, the success of greater than (or equal) threshold + hysteresis will transition
// the current state to AboveThreshold.
//
// To be inclusive means to include the limit value in each limit check.
// 'A' less than 'B', A < B, is not inclusive.  Less than or equal is inclusive, A <= B.
//
// The initial state is determined according to the ThresholdInitial policy.  A particular system
// may value events that go above or below the thresholds.  The initial state ensures that an important event
// was missed because the value never crossed a threshold throughout the existance of the class instance because
// the wrong initial hysteresis was applied.

/**
 * @brief Possible threshold states.
 */
enum class ThresholdState {
    Initial,            /**< Initial state */
    AboveThreshold,     /**< Input value is above threshold */
    BelowThreshold      /**< Input value is below threshold */
};

/**
 * @brief Convenient states for publishing
  */
enum class ThresholdAlert {
    None,
    Above,
    Below,
    Both,
};

/**
 * @brief Inclusive limit options for thresholds.
 * The the equal is also considered if a threshold (plus hysteresis) is inclusive
 *    above = (value >= threshold), value is greater than or equal to the threshold
 *    below = (value <= threshold), value is less than or equal to the threshold
 *
 * The equality statement is not considered when a threshold is not exclusive
 *    above = (value > threshold), value is greater than the threshold
 *    below = (value < threshold), value is less than the threshold
 */
enum class ThresholdInclusive {
    None,               /**< No limits are inclusive */
    Low,                /**< Only the low limit is inclusive */
    High,               /**< Only the high limit is inclusive */
    Both,               /**< Both limits are inclusive */
};

/**
 * @brief Initial state options for the first value evaluated.
 */
enum class ThresholdInitial {
    Low,                /**< Initial state is below threshold */
    High,               /**< Initial state is above threshold */
    Threshold,          /**< Initial state is determined by comparing input value to threshold */
};

/**
 * @brief Event handler for threshold crossings.
 *
 * @tparam T Type of the input value.
 * @param value The input value that triggered the event.
 * @param state The current threshold state.
 */
template<typename T>
using ThresholdEvent = std::function<void(T value, ThresholdState state)>;

/**
 * @brief Class for comparing an input value to a threshold.
 *
 * @tparam T Type of the input value.
 */
template<typename T>
class ThresholdComparator {
public:
    /**
     * @brief Constructor for ThresholdComparator.
     *
     * @param threshold The threshold value.
     * @param hysteresis The hysteresis value.  This value must be positive.
     * @param callback The callback function for threshold crossings.
     * @param inclusive Inclusive limit options.
     * @param initial Initial state options.
     */
    ThresholdComparator(
            T threshold,
            T hysteresis = {},
            ThresholdEvent<T> callback = nullptr,
            ThresholdInclusive inclusive = ThresholdInclusive::Both,
            ThresholdInitial initial = ThresholdInitial::Threshold) :
        _threshold(threshold),
        _thresholdMetCallback(callback),
        _inclusive(inclusive),
        _initial(initial){

        setHysteresis(hysteresis);
    }

    /**
     * @brief Set the threshold value.
     *
     * @param threshold The new threshold value.
     */
    void setThreshold(T threshold) {
        std::lock_guard<RecursiveMutex> lock(_mutex);
        _threshold = threshold;
        updateLimits();
    }

    /**
     * @brief Get the current threshold value.
     *
     * @return The current threshold value.
     */
    T getThreshold() const {
        std::lock_guard<RecursiveMutex> lock(_mutex);
        return _threshold;
    }

    /**
     * @brief Set the hysteresis value.  This value must be positive.
     *
     * @param hysteresis The new hysteresis value.
     */
    void setHysteresis(T hysteresis) {
        if (std::is_signed<T>::value && (T{} > hysteresis)) {
            hysteresis = -hysteresis;
        }

        std::lock_guard<RecursiveMutex> lock(_mutex);
        _hysteresis = hysteresis;
        updateInclusive(_inclusive);
        updateLimits();
    }

    /**
     * @brief Get the current hysteresis value.
     *
     * @return The current hysteresis value.
     */
    T getHysteresis() const {
        std::lock_guard<RecursiveMutex> lock(_mutex);
        return _hysteresis;
    }

    /**
     * @brief Set the threshold crossing callback function.
     *
     * @param callback The callback function for threshold crossings.
     */
    void setCallback(ThresholdEvent<T> callback) {
        std::lock_guard<RecursiveMutex> lock(_mutex);
        _thresholdMetCallback = callback;
    }

    /**
     * @brief Set the inclusive limit options.
     *
     * @param inclusive The new inclusive limit options.
     */
    void setInclusive(ThresholdInclusive inclusive) {
        std::lock_guard<RecursiveMutex> lock(_mutex);
        _inclusive = inclusive;
        updateInclusive(inclusive);
    }

    /**
     * @brief Get the current inclusive limit options.
     *
     * @return The current inclusive limit options.
     */
    ThresholdInclusive getInclusive() const {
        std::lock_guard<RecursiveMutex> lock(_mutex);
        return _inclusive;
    }

    /**
     * @brief Evaluate the input value against the threshold.
     *
     * @param inputValue The input value to compare against the threshold and hysteresis.
     * @return The current threshold state.
     */
    ThresholdState evaluate(T inputValue) {
        ThresholdEvent<T> callback {nullptr};
        auto currentState = evaluateValue(inputValue, callback);

        // The callback will be pointing to the appropriate callback if the threshold was crossed
        if (callback) {
            callback(inputValue, currentState);
        }

        return currentState;
    }

private:
    void updateLimits() {
        _low = _threshold - _hysteresis;
        _high = _threshold + _hysteresis;
    }

    void updateInclusive(ThresholdInclusive inclusive) {
        // Simplify decision making for limit checks
        switch (inclusive) {
            case ThresholdInclusive::Both:
                _inclusiveUseHigh = true;
                // Handle the special case where hysteresis is zero and both inclusive limits are set
                // we want to favor `above = (value >= threshold)`
                _inclusiveUseLow = ((T)0 == _hysteresis) ? false : true;
                break;

            case ThresholdInclusive::High:
                _inclusiveUseHigh = true;
                _inclusiveUseLow = false;
                break;

            case ThresholdInclusive::Low:
                _inclusiveUseHigh = false;
                _inclusiveUseLow = true;
                break;

            default: // ThresholdInclusive::None
                _inclusiveUseHigh = false;
                _inclusiveUseLow = false;
                break;
        }
    }

    void applyInitial(ThresholdInitial initial, T value) {
        switch (initial) {
            case ThresholdInitial::Threshold:
                _currentState = isAbove(value, _inclusiveUseHigh) ?
                                    ThresholdState::AboveThreshold : ThresholdState::BelowThreshold;
                break;

            case ThresholdInitial::High:
                _currentState = ThresholdState::AboveThreshold;
                break;

            case ThresholdInitial::Low:
                _currentState = ThresholdState::BelowThreshold;
                break;
        }
    }

    inline bool isBelowWithHysteresis(T value, bool inclusive) {
        return (inclusive) ? (value <= _low) : (value < _low);
    }

    inline bool isAboveWithHysteresis(T value, bool inclusive) {
        return (inclusive) ? (value >= _high) : (value > _high);
    }

    inline bool isBelow(T value, bool inclusive) {
        return (inclusive) ? (value <= _threshold) : (value < _threshold);
    }

    inline bool isAbove(T value, bool inclusive) {
        return (inclusive) ? (value >= _threshold) : (value > _threshold);
    }

    ThresholdState evaluateValue(T inputValue, ThresholdEvent<T>& cb) {
        std::lock_guard<RecursiveMutex> lock(_mutex);
        auto above = isAboveWithHysteresis(inputValue, _inclusiveUseHigh);
        auto below = isBelowWithHysteresis(inputValue, _inclusiveUseLow);
        auto changed {false};

        switch (_currentState) {
            case ThresholdState::AboveThreshold:
                if (below) {
                    _currentState = ThresholdState::BelowThreshold;
                    changed = true;
                }
                break;

            case ThresholdState::BelowThreshold:
                if (above) {
                    _currentState = ThresholdState::AboveThreshold;
                    changed = true;
                }
                break;

            case ThresholdState::Initial:
                applyInitial(_initial, inputValue);
                changed = true;
                break;

        }

        if (_thresholdMetCallback && changed) {
            cb = _thresholdMetCallback;
        }

        return _currentState;
    }

    RecursiveMutex _mutex;
    T _threshold;
    T _hysteresis {};
    ThresholdEvent<T> _thresholdMetCallback;
    ThresholdInclusive _inclusive;
    bool _inclusiveUseHigh {};
    bool _inclusiveUseLow {};
    T _low {};
    T _high {};

    ThresholdInitial _initial;
    ThresholdState _currentState {ThresholdState::Initial};
};
