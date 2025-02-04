package org.firstinspires.ftc.teamcode.system.hardware;

import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.IntakeTurretServoState.STRAIGHT;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.system.accessory.pids.PID;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;
import org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers.CRServoPika;
import org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers.MotorPika;
import org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers.ServoPika;

@Config
public class IntakeSubsystem
{
    ServoPika clipS, armS, turretS;
    CRServoPika intakeS;
    MotorPika intakeSlideMotor;
    RevColorSensorV3 colourSensor;
    public static double kP = 0.04, kI = 0, kD = 0;
    PID intakeSlidesPID = new PID(kP, kI, kD, 0, 0);
    public int slideTarget, slidePosition;
    long colorValue;
    GeneralHardware.Side side;

    public static final double // in inches
        slideExtensionLimit = 18.5,
        slideTeleClose = 12,
        slideTeleFar = 18.5, // max extension is 27 under the extension limit
        slideTeleBase = 0,
        slideTransfer = -2,
        slideAutoFar = 18.5,
        slideAutoClose = 14;
    private final double slideThreshold = 8;

    public enum IntakeSpinState
    {
        INTAKE(1),
        REVERSE(-1),
        OFF(0);

        public final double power;

        IntakeSpinState(double power)
        {
            this.power = power;
        }
    }
    public enum IntakeArmServoState
    {
        READY(0),
        TRANSFER_BACK(0),
        TRANSFER_FRONT(0),
        HALF_TRANSFER(0),
        HORIZONTAL(0),
        HALF_DOWN(0),
        DOWN(0),
        BACK(0),
        IN(0),
        AROUND(0);

        public final double pos;

        IntakeArmServoState(double pos)
        {
            this.pos = pos;
        }
    }
    public enum IntakeClipServoState
    {
        HOLD(0),
        OPEN(0);

        public final double pos;

        IntakeClipServoState(double pos)
        {
            this.pos = pos;
        }

    }
    public enum IntakeTurretServoState
    {
        MAX_LEFT(0),
        LEFT(0),
        STRAIGHT(0),
        RIGHT(0),
        MAX_RIGHT(0),
        AROUND(0);

        public final double pos;

        IntakeTurretServoState(double pos)
        {
            this.pos = pos;
        }
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
    public boolean isRed = false, isYellow = false;

    public IntakeSubsystem(GeneralHardware hardware)
    {
        side = hardware.side;

        turretS = hardware.turretS;
        intakeS = hardware.intakeS;
        clipS = hardware.clipS;
        colourSensor = hardware.colourSensor; // no supplier as i want this to pool immediately and synchronously

        intakeSlideMotor = hardware.intakeSlidesM;

        intakeHardwareSetUp(); // this can now be called from here because the objects initialize at hardware
    }

    public void intakeHardwareSetUp()
    {
        // if we need to reverse anything
        intakeSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void intakeReads(boolean i2c)
    {
        slidePosition = intakeSlideMotor.getCurrentPosition();
        if (i2c)
        {
            double redValue = colourSensor.getNormalizedColors().red;
            double blueValue = colourSensor.getNormalizedColors().blue;
            isRed = redValue > blueValue;
            colorValue = colourSensor.alpha();
            isYellow = colorValue > 2500;
        }
    }

    public void intakeSpin(IntakeSpinState state)
    {
        switch (state)
        {
            case INTAKE:
                intakeS.setPower(1);
                break;
            case OFF:
                intakeS.setPower(0);
                break;
            case REVERSE:
                intakeS.setPower(-1);
                break;
        }
    }
    public void intakeSpin(double spinDirection)
    {
        intakeS.setPower(spinDirection);
    }

    public void intakeArm(IntakeArmServoState state)
    {
        armS.setPosition(state.pos);
    }
    public void armSetPos(double armPos)
    {
        armS.setPosition(armPos);
    }
    public double armGetPos()
    {
        return armS.getPosition();
    }

    public void intakeTurret(IntakeTurretServoState state)
    {
        turretS.setPosition(state.pos);
    }
    public void intakeTurretSetPos(double pos)
    {
        turretS.setPosition(pos);
    }
    public void intakeTurretSetAngle(double angle)
    {
        turretS.setPosition(angle * 0.002816901408); // 1 / 355
    }
    public void intakeTurretBasedOnHeadingVel(double headingVel)
    {
        double t = Math.min(1, headingVel / 2);
        double i = interpolation(0, 65, t);
        turretS.setPosition(STRAIGHT.pos + angleToServoTicks(i) * Math.signum(headingVel));
    }
    public double angleToServoTicks(double angle)
    {
        return angle * (1/355.0);
    }
    private double interpolation(double p1, double p2, double t) {
        return (1 - t) * p1 + t * p2;
    }
    public double getTurretAngle()
    {
        return turretS.getPosition() * 355;
    }
    public double getTurretPos()
    {
        return turretS.getPosition();
    }
    public void intakeClip(IntakeClipServoState state)
    {
        clipS.setPosition(state.pos);
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
    public void intakeSlideInternalPID(double inches, boolean limit)
    {
        slideTarget = (int) inchesToTicksSlidesMotor(inches);
        if (slideTarget > slideExtensionLimit) slideTarget = (int) inchesToTicksSlidesMotor(slideExtensionLimit);
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
                if (isYellow) return false;
                else return isRed ? side == GeneralHardware.Side.Red : side == GeneralHardware.Side.Blue;
            case NEUTRAL:
                return (isYellow) ||
                        isRed ? side == GeneralHardware.Side.Red : side == GeneralHardware.Side.Blue;
            case YELLOW_ONLY:
                return isYellow;
            default:
                return true;
        }
//
//        if (intakeFilter == IntakeFilter.OFF) return true;
//        if (colorValue > 1500) return intakeFilter != IntakeFilter.SIDE_ONLY;
//        else if (!isRed) return (side == GeneralHardware.Side.Blue);
//        else return (side == GeneralHardware.Side.Red);
    }
    public boolean isSample()
    {
        if (isYellow) return true;
        else return isRed ? side == GeneralHardware.Side.Red : side == GeneralHardware.Side.Blue;
    }
    public double getColorValue()
    {
        return colorValue;
    }
    public GeneralHardware.Side getSide()
    {
        return side;
    }
    public boolean slideReached(double slideTargetIn)
    {
        return Math.abs(inchesToTicksSlidesMotor(slideTargetIn) - slidePosition) < slideThreshold;
    }
    public boolean slideOverPosition(double slideTargetIn)
    {
        return slidePosition > inchesToTicksSlidesMotor(slideTargetIn) ;
    }
    public boolean isSlidesAtBase()
    {
        return slidePosition < 6; // this works as slide base is 0
    }

    public double ticksToInchesSlidesMotor(double ticks){
        return ((1.005007874 * 2 * Math.PI) / (TICKS_PER_BAREMOTOR * 5.6428571429)) * ticks;
    }

    public double inchesToTicksSlidesMotor (double inches){
        return ((TICKS_PER_BAREMOTOR * 5.6428571429)/(1.005007874 * 2 * Math.PI)) * inches; //ticks per inches
        // ratio is 70/12 = 5
    }

}
