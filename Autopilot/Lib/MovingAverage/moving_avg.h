/*
 * moving_avg.h
 *
 *  Created on: Feb. 11, 2025
 *      Author: jeffr
 */

#ifndef LIB_MOVINGAVERAGE_MOVING_AVG_H_
#define LIB_MOVINGAVERAGE_MOVING_AVG_H_

#include <cstddef>

class MovingAverage {
public:
    explicit MovingAverage(size_t windowSize, float* buffer);
    void add(float value);
    float getAverage() const;
    bool getFilled() const;

private:
    float* buffer;
    size_t windowSize;
    size_t index;
    float sum;
    bool filled;
};

#endif /* LIB_MOVINGAVERAGE_MOVING_AVG_H_ */
