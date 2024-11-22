package org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.gvf.MecanumDrive;

public class MotorPika
{
    private DcMotorEx motor;
    private double power, prevPower;
    private static double EPSILON_DELTA = 0.005;

    public MotorPika(DcMotorEx motor)
    {
        this.motor = motor;
    }

    public void setPower(double pow)
    {
        if (
                (Math.abs(pow - prevPower) > EPSILON_DELTA) ||
                        (pow == 0.0 && prevPower != 0.0) ||
                        (pow >= 1.0 && !(prevPower >= 1.0)) ||
                        (pow <= -1.0 && !(prevPower <= -1.0))
        )
        {
            prevPower = pow;
            motor.setPower(pow);
        }
    }
    public int getCurrentPosition()
    {
        return motor.getCurrentPosition();
    }
    public double getCurrent(CurrentUnit unit)
    {
        return motor.getCurrent(unit);
    }
    public void setMode(DcMotor.RunMode mode)
    {
        motor.setMode(mode);
    }
    public void setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior zeroPowerBehaviour)
    {
        motor.setZeroPowerBehavior(zeroPowerBehaviour);
    }
    public void resetEncoder()
    {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior)
    {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setTargetPosition(int targetPosition)
    {
        motor.setTargetPosition(targetPosition);
    }

    public DcMotorEx getMotor()
    {
        return motor;
    }
    public void setDirection(DcMotorSimple.Direction direction)
    {
        motor.setDirection(direction);
    }
}

