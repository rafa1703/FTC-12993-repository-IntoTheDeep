package org.firstinspires.ftc.teamcode.system.hardware;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.motorCaching;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.system.accessory.ToggleUpOrDown;
import org.firstinspires.ftc.teamcode.system.accessory.pids.PID;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

public class IntakeSubsystem
{
    ServoImplEx chuteS, flapS, leftArmS, rightArmS;

    DcMotorEx intakeMotor, intakeSlideMotor;
    ColorSensor colorSensor;
    PID intakeSlidesPID = new PID(0.04, 0, 0.003, 0, 0);

    int slideTarget, slidePosition;
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
            flapDownPos = 0;

    public static final int
        slideTeleClose = 200,
        slideTeleFar = 400;
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
        TRANSFER,
        DOWN
    }
    public IntakeSpinState intakeSpinState;

    public IntakeSubsystem(GeneralHardware hardware)
    {
        side = hardware.side;

        chuteS = hardware.sch0;
        flapS = hardware.sch1;
        leftArmS = hardware.sch2;
        rightArmS = hardware.sch3;
        colorSensor = hardware.cs0; // no supplier as i want this to pool immediately and synchronous

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

    public void intakeChuteState(IntakeChuteServoState state)
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
    public void intakeFlapState(IntakeFlapServoState state)
    {
        switch (state)
        {
            case TRANSFER:
                flapS.setPosition(flapTransferPos);
                break;
            case DOWN:
                flapS.setPosition(flapDownPos);
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
    public void intakeSlideMotorEncodersReset(){
        intakeSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void intakeSlideMotorRawControl(double manualcontrolintakeslide)
    {
        intakeSlideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeSlideMotor.setPower(manualcontrolintakeslide * 0.75);
    }
    public double getColorValue()
    {
        return colorValue;
    }
    public GeneralHardware.Side getSide()
    {
        return side;
    }


}
