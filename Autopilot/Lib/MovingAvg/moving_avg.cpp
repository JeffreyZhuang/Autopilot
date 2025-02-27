#include <Lib/MovingAvg/moving_avg.h>

MovingAverage::MovingAverage(size_t windowSize, float* buffer)
    : buffer(buffer), windowSize(windowSize), index(0), sum(0), filled(false) {
    for (size_t i = 0; i < windowSize; ++i) {
        buffer[i] = 0.0f;
    }
}

void MovingAverage::add(float value) {
    sum -= buffer[index];
    buffer[index] = value;
    sum += value;
    index = (index + 1) % windowSize;
    if (index == 0) filled = true;
}

float MovingAverage::getAverage() const {
    size_t count = filled ? windowSize : index;
    return count == 0 ? 0.0f : sum / count;
}

bool MovingAverage::getFilled() const {
	return filled;
}
