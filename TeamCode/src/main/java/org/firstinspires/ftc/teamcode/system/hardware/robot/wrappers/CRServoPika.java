package org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class CRServoPika
{
    private CRServoImplEx servo;
    double prevPower = -1;
    private static double EPSILON_DELTA = 0.005;

    public CRServoPika(CRServoImplEx servo)
    {
        this.servo = servo;
    }
    public void setPower(double pow)
    { // so this is just a wrapper for this tbh
        if (
                (Math.abs(pow - prevPower) > EPSILON_DELTA) ||
                        (pow == 0.0 && prevPower != 0.0) ||
                        (pow >= 1.0 && !(prevPower >= 1.0)) ||
                        (pow <= -1.0 && !(prevPower <= -1.0))
        )
        {
            servo.setPower(pow);
            prevPower = pow;
        }
    }

    public double getPower()
    {
        return servo.getPower();
    }
    public void setDirection(DcMotor.Direction direction)
    {
        servo.setDirection(direction);
    }
    public void setPwmDisable()
    {
        servo.setPwmDisable();
    }
    public void setPwmEnable()
    {
        servo.setPwmEnable();
    }
    public boolean isPwmEnabled()
    {
       return servo.isPwmEnabled();
    }
}
