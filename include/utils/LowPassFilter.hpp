#ifndef LOW_PASS_FILTER_HPP_
#define LOW_PASS_FILTER_HPP_

namespace utils {

/**
 * @brief Simple First-order Low Pass Filter (IIR)
 * Formula: y[k] = alpha * x[k] + (1 - alpha) * y[k-1]
 */
class LowPassFilter {
private:
    float alpha_;
    float last_output_;
    bool initialized_;

public:
    /**
     * @param alpha Filter coefficient (0.0 to 1.0). 
     *              Smaller value means more filtering (smoother but slower).
     */
    explicit LowPassFilter(float alpha = 1.0f) 
        : alpha_(alpha), last_output_(0.0f), initialized_(false) {}

    /**
     * @brief Apply filter to a new sample
     */
    float update(float input) {
        if (!initialized_) {
            last_output_ = input;
            initialized_ = true;
            return last_output_;
        }
        last_output_ = alpha_ * input + (1.0f - alpha_) * last_output_;
        return last_output_;
    }

    /**
     * @brief Apply filter to angular data (handles wrap-around like -PI to PI)
     * This is specialized for Yaw/Heading.
     */
    float updateAngle(float input) {
        if (!initialized_) {
            last_output_ = input;
            initialized_ = true;
            return last_output_;
        }

        // Handle wrap-around
        float diff = input - last_output_;
        if (diff > 3.14159265f) diff -= 2.0f * 3.14159265f;
        if (diff < -3.14159265f) diff += 2.0f * 3.14159265f;

        last_output_ = last_output_ + alpha_ * diff;

        // Keep last_output in [-PI, PI]
        if (last_output_ > 3.14159265f) last_output_ -= 2.0f * 3.14159265f;
        if (last_output_ < -3.14159265f) last_output_ += 2.0f * 3.14159265f;

        return last_output_;
    }

    void setAlpha(float alpha) { alpha_ = alpha; }
    void reset() { initialized_ = false; }
};

} // namespace utils

#endif /* LOW_PASS_FILTER_HPP_ */
