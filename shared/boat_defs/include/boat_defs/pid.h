#pragma once

namespace boat {
    class SimplePid {
    public:
        SimplePid() : kp_(0.0), ki_(0.0), kd_(0.0), prev_error_(0.0), integral_(0.0), first_(true) {}

        SimplePid(double kp, double ki, double kd) : kp_(kp), ki_(ki), kd_(kd), prev_error_(0.0), integral_(0.0), first_(true) {}

        void reset() {
            prev_error_ = 0.0;
            integral_ = 0.0;
            first_ = true;
        }

        double update(double error, double dt) {
            integral_ += error * dt;
            double derivative = first_ ? 0.0 : (error - prev_error_) / dt;
            prev_error_ = error;
            first_ = false;
            return kp_ * error + ki_ * integral_ + kd_ * derivative;
        }
    private:
        double kp_;
        double ki_;
        double kd_;
        double prev_error_;
        double integral_;

        bool first_;
    };
}