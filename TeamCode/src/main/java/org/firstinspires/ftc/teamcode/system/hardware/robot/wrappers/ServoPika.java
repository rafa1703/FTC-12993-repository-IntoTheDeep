package org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;

public class ServoPika
{
    private ServoImplEx servo;
    double prevPos;
    private static double EPSILON_DELTA = 0.005;

    public ServoPika(ServoImplEx servo)
    {
        this.servo = servo;
    }
    public synchronized void setPosition(double position)
    { // so this is just a wrapper for this tbh
        if (
        (Math.abs(position - prevPos) > EPSILON_DELTA) ||
                (position == 0.0 && prevPos != 0.0) ||
                (position >= 1.0 && !(prevPos >= 1.0))
        )
        {
            servo.setPosition(position);
            prevPos = position;
        }
    }

    public double getPosition()
    {
        return servo.getPosition();
    }
}
