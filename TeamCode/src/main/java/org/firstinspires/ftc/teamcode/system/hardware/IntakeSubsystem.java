package org.firstinspires.ftc.teamcode.system.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.system.accessory.pids.PID;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;
import org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers.MotorPika;
import org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers.ServoPika;

@Config
public class IntakeSubsystem
{
    ServoPika chuteS, flapS, leftArmS, rightArmS, clipS;

    MotorPika intakeMotor, intakeSlideMotor;
    RevColorSensorV3 colorSensor;
    public static double kP = 0.04, kI = 0, kD = 0;
    PID intakeSlidesPID = new PID(kP, kI, kD, 0, 0);
    public int slideTarget, slidePosition;
    long colorValue;
    double intakeSpeed = 1;
    GeneralHardware.Side side;

    public static final double
        leftArmHighPos = 0.1,
        leftArmDropHighPos = 0,
        leftArmLowPos = 0.23;
    public static final double
        rightArmHighPos = 0.1,
        rightArmDropHighPos = 0,
        rightArmLowPos = 0.2;

    public static final double
        chuteUpPos = 0.05,
        chuteDropPos = 0.6;
    public static final double
        flapTransferPos = 0,
        flapDownPos = 1,
        flapReadyPos = 1;

    public static final int // in inches
        slideTeleClose = 14,
        slideTeleFar = 20, // max extension is 27 under the extension limit
        slideTeleBase = 0,
        slideTransfer = -2;
    public final double
        clipHoldPos = 0.7,
        clipOpenPos = 1;
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
        DROP_HIGH,
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
        SIDE_ONLY, // Only alliance samples
        NEUTRAL, // both alliance and neutral samples
        YELLOW_ONLY,
        OFF // intakes everything
    }
    public IntakeFilter intakeFilter = IntakeFilter.NEUTRAL;

    private final double TICKS_PER_BAREMOTOR = 28;
    private boolean isRed = false;

    public IntakeSubsystem(GeneralHardware hardware)
    {
        side = hardware.side;

        chuteS = hardware.chuteS;
        flapS = hardware.flapS;
        leftArmS = hardware.intakeLeftArmS;
        rightArmS = hardware.intakeRightArmS;
        clipS = hardware.clipS;
        colorSensor = hardware.cs0; // no supplier as i want this to pool immediately and synchronously

        intakeMotor = hardware.intakeM;
        intakeSlideMotor = hardware.intakeSlidesM;

        intakeHardwareSetUp(); // this can now be called from here because the objects initialize at hardware
    }

    public void intakeHardwareSetUp()
    {
        // if we need to reverse anything
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void intakeReads(boolean i2c)
    {
        slidePosition = intakeSlideMotor.getCurrentPosition();
        //if (i2c) colorValue = colorSensor.alpha();
        if (i2c)
        {
            double redValue = colorSensor.getNormalizedColors().red;
            double blueValue = colorSensor.getNormalizedColors().blue;
            isRed = !(blueValue > redValue);
            colorValue = colorSensor.alpha();
        }
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
                intakeMotor.setPower(-0.7);
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
            case DROP_HIGH:
                leftArmS.setPosition(leftArmDropHighPos);
                rightArmS.setPosition(rightArmDropHighPos);
                break;
            case LOW:
                leftArmS.setPosition(leftArmLowPos);
                rightArmS.setPosition(rightArmLowPos);
                break;
        }
    }
    public void armSetPos(double rPos, double lPos)
    {
        rightArmS.setPosition(rPos);
        leftArmS.setPosition(lPos);
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
    public void chuteSetPos(double pos)
    {
        chuteS.setPosition(pos);
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
    public void flapSetPos(double pos)
    {
        flapS.setPosition(pos);
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
    public void clipSetPos(double pos)
    {
        clipS.setPosition(pos);
    }

    public void intakeSlideTo(int rotations)
    {
        slideTarget = rotations;
        intakeSlideMotor.setPower(intakeSlidesPID.update(rotations, slidePosition, 1));
    }
    public void intakeSlideInternalPID(double inches)
    {
        slideTarget = (int) inchesToTicksSlidesMotor(inches);
        intakeSlideMotor.setTargetPosition(slideTarget);
        intakeSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeSlideMotor.setPower(1);
    }
    public void intakeSlideInternalPIDTicks(int ticks)
    {
        slideTarget = ticks;
        intakeSlideMotor.setTargetPosition(slideTarget);
        intakeSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeSlideMotor.setPower(1);
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
        switch (intakeFilter)
        {
            case SIDE_ONLY:
                if (colorValue > 1500) return false;
                else return isRed ? side == GeneralHardware.Side.Red : side == GeneralHardware.Side.Blue;
            case NEUTRAL:
                return (colorValue > 1500) || isRed ? side == GeneralHardware.Side.Red : side == GeneralHardware.Side.Blue;
            case YELLOW_ONLY:
                return (colorValue > 1500);
            default:
                return true;
        }
//
//        if (intakeFilter == IntakeFilter.OFF) return true;
//        if (colorValue > 1500) return intakeFilter != IntakeFilter.SIDE_ONLY;
//        else if (!isRed) return (side == GeneralHardware.Side.Blue);
//        else return (side == GeneralHardware.Side.Red);
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
    public boolean isSlidesAtBase()
    {
        return slidePosition < slideThreshold; // this works as slide base is 0
    }

    public double ticksToInchesSlidesMotor(double ticks){
        return ((1.005007874 * 2 * Math.PI) / (TICKS_PER_BAREMOTOR * 5.6428571429)) * ticks;
    }

    public double inchesToTicksSlidesMotor (double inches){
        return ((TICKS_PER_BAREMOTOR * 5.6428571429)/(1.005007874 * 2 * Math.PI)) * inches; //ticks per inches
        // ratio is 70/12 = 5
    }


}
