package org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;

public class ServoPika extends ServoImplEx
{
    private ServoImplEx servo;
    double prevPos;
    private static double EPSILON_DELTA = 0.005;
    public ServoPika(ServoControllerEx controller, int portNumber, @NonNull ServoConfigurationType servoType)
    {
        super(controller, portNumber, servoType);
    }

    public ServoPika(ServoControllerEx controller, int portNumber, Direction direction, @NonNull ServoConfigurationType servoType)
    {
        super(controller, portNumber, direction, servoType);
    }

    @Override
    public synchronized void setPosition(double position)
    { // so this is just a wrapper for this tbh
        if (
        (Math.abs(position - prevPos) > EPSILON_DELTA) ||
                (position == 0.0 && prevPos != 0.0) ||
                (position >= 1.0 && !(prevPos >= 1.0)) ||
                (position <= -1.0 && !(prevPos <= -1.0))
        )
        {
            super.setPosition(position);
            prevPos = position;
        }
    }
}
