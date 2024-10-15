package org.firstinspires.ftc.teamcode.system.hardware;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.system.accessory.pids.PID;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

public class IntakeSubsystem
{
    ServoImplEx chuteS, flapS, leftArmS, rightArmS, clipS;

    DcMotorEx intakeMotor, intakeSlideMotor;
    ColorSensor colorSensor;
    PID intakeSlidesPID = new PID(0.04, 0, 0.003, 0, 0);

    public int slideTarget, slidePosition;
    double colorValue;
    double intakeSpeed = 1;
    GeneralHardware.Side side;

    public static final double
        leftArmHighPos = 0,
        leftArmLowPos = 1;
    public static final double
        rightArmHighPos = 1,
        rightArmLowPos = 0;

    public static final double
        chuteUpPos = 1,
        chuteDropPos = 0;
    public static final double
        flapTransferPos = 1,
        flapDownPos = 0.5,
        flapReadyPos = 0;

    public static final int
        slideTeleClose = 200,
        slideTeleFar = 400,
        slideTeleBase = 0,
        slideTeleTransfer = -200;
    public final double
        clipHoldPos = 1,
        clipOpenPos = 0;
    private final double slideThreshold = 8;
    public enum IntakeSpinState
    {
        INTAKE,
        DROP,
        REVERSE,
        OFF
    }
    public enum IntakeArmServoState
    {
        HIGH,
        LOW
    }
    public enum IntakeChuteServoState
    {
        UP,
        DROP
    }
    public enum IntakeFlapServoState
    {
        READY,
        TRANSFER,
        DOWN
    }
    public enum IntakeClipServoState
    {
        HOLD,
        OPEN
    }
    public enum IntakeFilter
    {
        SIDE_SPECIFIC, // Only alliance samples
        NEUTRAL, // both alliance and neutral samples
        OFF // intakes everything
    }
    public IntakeFilter intakeFilter = IntakeFilter.NEUTRAL;

    public IntakeSubsystem(GeneralHardware hardware)
    {
        side = hardware.side;

        chuteS = hardware.sch0;
        flapS = hardware.sch1;
        leftArmS = hardware.sch2;
        rightArmS = hardware.sch3;
        clipS = hardware.sch4;
        colorSensor = hardware.cs0; // no supplier as i want this to pool immediately and synchronously

        intakeMotor = hardware.mch0;
        intakeSlideMotor = hardware.mch1;

        intakeHardwareSetUp(); // this can now be called from here because the objects initialize at hardware
    }

    public void intakeHardwareSetUp()
    {
        // if we need to reverse anything
        intakeSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void intakeReads(boolean i2c)
    {
        slidePosition = intakeSlideMotor.getCurrentPosition();
        if (i2c) colorValue = colorSensor.alpha();
    }

    public void intakeSpin(IntakeSpinState state)
    {
        switch (state)
        {

            case INTAKE:
                intakeMotor.setPower(intakeSpeed);
                break;
            case DROP:
            case OFF:
                intakeMotor.setPower(0);
                break;
            case REVERSE:
                intakeMotor.setPower(-0.6);
                break;
        }
    }
    public void intakeSpin(double spinDirection)
    {
        intakeMotor.setPower(spinDirection);
    }

    public void intakeArm(IntakeArmServoState state)
    {
        switch (state)
        {
            case HIGH:
                leftArmS.setPosition(leftArmHighPos);
                rightArmS.setPosition(rightArmHighPos);
                break;
            case LOW:
                leftArmS.setPosition(leftArmLowPos);
                rightArmS.setPosition(rightArmLowPos);
                break;
        }
    }

    public void intakeChute(IntakeChuteServoState state)
    {
        switch (state)
        {
            case UP:
                chuteS.setPosition(chuteUpPos);
                break;
            case DROP:
                chuteS.setPosition(chuteDropPos);
                break;
        }
    }
    public void intakeFlap(IntakeFlapServoState state)
    {
        switch (state)
        {
            case READY:
                flapS.setPosition(flapReadyPos);
                break;
            case TRANSFER:
                flapS.setPosition(flapTransferPos);
                break;
            case DOWN:
                flapS.setPosition(flapDownPos);
                break;
        }
    }
    public void intakeClip(IntakeClipServoState state)
    {
        switch (state)
        {
            case HOLD:
                clipS.setPosition(clipHoldPos);
                break;
            case OPEN:
                clipS.setPosition(clipOpenPos);
                break;
        }
    }

    public void intakeSlideTo(int rotations)
    {
        slideTarget = rotations;
        intakeSlideMotor.setPower(intakeSlidesPID.update(rotations, slidePosition, 1));
    }
    public void intakeSlideInternalPID(int rotations)
    {
        slideTarget = rotations;
        intakeSlideMotor.setTargetPosition(slideTarget);
        intakeSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void intakeSlideInternalPID(int rotations, double maxPower)
    {
        slideTarget = rotations;
        intakeSlideMotor.setTargetPosition(slideTarget);
        intakeSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeSlideMotor.setPower(maxPower);
    }
    public void intakeSlideMotorEncoderReset()
    {
        intakeSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void intakeSlideMotorRawControl(double manualControlIntakeSlide)
    {
        intakeSlideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeSlideMotor.setPower(manualControlIntakeSlide * 0.75);
    }
    public boolean colorLogic()
    {
        if (intakeFilter == IntakeFilter.OFF) return true;
        if (colorValue > 5000) return (side == GeneralHardware.Side.Red); // assuming 5000 is red lower threshold
        else if (colorValue < 3000) return (side == GeneralHardware.Side.Blue);  // assuming 3000 is yellow lower threshold
        else return intakeFilter != IntakeFilter.SIDE_SPECIFIC;
    }
    public double getColorValue()
    {
        return colorValue;
    }
    public GeneralHardware.Side getSide()
    {
        return side;
    }
    public boolean slideReached(int slideTarget)
    {
        return Math.abs(slideTarget - slidePosition) < slideThreshold;
    }


}
