#pragma once

class PIDController {
    double kP;
    double kD;
    double kI;

    double prevTime;
    double integral;
    double prevError;

public:
    PIDController(double kP, double kI, double kD)
        : kP(kP), kD(kD), kI(kI), prevTime(0), integral(0), prevError(0)
    {}

    double update(double current, double target, double timeStamp) {
        double error = target - current;
        double dt = timeStamp - prevTime;
        double P = error * kP;
        double D = (error - prevError) / dt * kD;
        integral += error * dt;
        double I = integral * kI;

        prevError = error;
        prevTime = timeStamp;

        return (P + I + D);
    }
};