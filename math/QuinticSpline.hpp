#pragma once

#include <cmath>

class QuinticSegment {
public:
    double p0_, v0_, a0_, p1_, v1_, a1_;

    QuinticSegment(double p0, double v0, double a0, double p1, double v1, double a1) 
        : p0_(p0), v0_(v0), a0_(a0), p1_(p1), v1_(v1), a1_(a1)
    {}

    double pos(double t) const {
        return (H_p0(t) * p0_ + H_v0(t) * v0_ + H_a0(t) * a0_ + H_p1(t) * p1_ + H_v1(t) * v1_ + H_a1(t) * a1_);
    }

    double vel(double t) const {
        return (dH_p0(t) * p0_ + dH_v0(t) * v0_ + dH_a0(t) * a0_ + dH_p1(t) * p1_ + dH_v1(t) * v1_ + dH_a1(t) * a1_);
    }

    double acc(double t) const {
        return (ddH_p0(t) * p0_ + ddH_v0(t) * v0_ + ddH_a0(t) * a0_ + ddH_p1(t) * p1_ + ddH_v1(t) * v1_ + ddH_a1(t) * a1_);
    }
    
private:
    // Quintic Hermite spline basis functions
    static constexpr double H_p0(double t) { return 1 - 10*t*t*t + 15*t*t*t*t - 6*t*t*t*t*t; }
    static constexpr double H_p1(double t) { return 10*t*t*t - 15*t*t*t*t + 6*t*t*t*t*t; }
    static constexpr double H_v0(double t) { return t - 6*t*t*t + 8*t*t*t*t - 3*t*t*t*t*t; }
    static constexpr double H_v1(double t) { return -4*t*t*t + 7*t*t*t*t - 3*t*t*t*t*t; }
    static constexpr double H_a0(double t) { return 0.5*t*t - 1.5*t*t*t + 1.5*t*t*t*t - 0.5*t*t*t*t*t; }
    static constexpr double H_a1(double t) { return 0.5*t*t*t - t*t*t*t + 0.5*t*t*t*t*t; }

    // First derivatives
    static constexpr double dH_p0(double t) { return -30*t*t + 60*t*t*t - 30*t*t*t*t; }
    static constexpr double dH_p1(double t) { return  30*t*t - 60*t*t*t + 30*t*t*t*t; }
    static constexpr double dH_v0(double t) { return 1 - 18*t*t + 32*t*t*t - 15*t*t*t*t; }
    static constexpr double dH_v1(double t) { return -12*t*t + 28*t*t*t - 15*t*t*t*t; }
    static constexpr double dH_a0(double t) { return t - 4.5*t*t + 6*t*t*t - 2.5*t*t*t*t; }
    static constexpr double dH_a1(double t) { return 1.5*t*t - 4*t*t*t + 2.5*t*t*t*t; }

    // Second derivatives
    static constexpr double ddH_p0(double t) { return -60*t + 180*t*t - 120*t*t*t; }
    static constexpr double ddH_p1(double t) { return  60*t - 180*t*t + 120*t*t*t; }
    static constexpr double ddH_v0(double t) { return -36*t + 96*t*t - 60*t*t*t; }
    static constexpr double ddH_v1(double t) { return -24*t + 84*t*t - 60*t*t*t; }
    static constexpr double ddH_a0(double t) { return 1 - 9*t + 18*t*t - 10*t*t*t; }
    static constexpr double ddH_a1(double t) { return 3*t - 12*t*t + 10*t*t*t; }  
};