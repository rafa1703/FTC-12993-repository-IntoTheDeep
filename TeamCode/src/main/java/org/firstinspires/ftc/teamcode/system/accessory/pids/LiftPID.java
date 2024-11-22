package org.firstinspires.ftc.teamcode.system.accessory.pids;

import androidx.core.math.MathUtils;

import com.acmerobotics.roadrunner.util.MathUtil;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LiftPID
{
    ElapsedTime timer = new ElapsedTime();
    double Kp;
    double Ki;
    double Kd;
    double integralSumLimit; // max integral value, should have its own constructor variable to change
    double lastError;
    double integralSum;
    double error;
    double derivative;
    double output;
    double position;
    double targetPosition;


    public LiftPID(double Kp, double Ki, double Kd, double integralSumLimit){
        this.Kp = Kp;
        this.Kd = Kd;
        this.Ki = Ki;
        this.integralSumLimit = integralSumLimit;
    }

    public double update(double target, double pos, double kf) {
        position = pos;
        targetPosition = target;
        error = target-pos;
        derivative = (error-lastError) / timer.seconds();
        integralSum = integralSum + (error * timer.seconds());
        // set a limit on our integral sum
        if (integralSum > integralSumLimit) {
            integralSum = integralSumLimit;
        }
        if (integralSum < -integralSumLimit) {
            integralSum = -integralSumLimit;
        }
        output = (Kp * error) + (Ki * integralSum) + (Kd * derivative) + kf;
        output = MathUtils.clamp(output, -1.0, 1.0);

        lastError = error;
        timer.reset(); // i didn't do this it will mess with the PIDs

        return output;
    }

}
