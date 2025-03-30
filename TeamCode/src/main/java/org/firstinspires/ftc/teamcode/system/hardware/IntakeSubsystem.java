package org.firstinspires.ftc.teamcode.system.hardware;

import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.IntakeTurretServoState.STRAIGHT;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.firstinspires.ftc.teamcode.gvf.utils.Vector;
import org.firstinspires.ftc.teamcode.system.accessory.pids.PID;
import org.firstinspires.ftc.teamcode.system.accessory.supplier.TimedSupplier;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;
import org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers.CRServoPika;
import org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers.MotorPika;
import org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers.ServoPika;

@Config
public class IntakeSubsystem
{

    ServoPika clipS, armS, turretS;
    CRServoPika intakeCS;
    MotorPika intakeSlideMotor;
    RevColorSensorV3 colourSensor;
    TimedSupplier<Integer> colourSupplier;
    DistanceSensor distanceSensor;
    TimedSupplier<Double> distanceSupplier;
    public static double kP = 0.04, kI = 0, kD = 0;
    PID intakeSlidesPID = new PID(kP, kI, kD, 0, 0);
    public int slideTarget, slidePosition;
    double colorValue;
    double colourSensorDistance;
    GeneralHardware.Side side;
    double intakeSlidesFineAdjustTimer;

    public static final double // in inches, 24 max slides
            slideExtensionLimit = 26,
            slideTeleClose = 12,
            slideTeleFar = 23.2,
            slideTeleBase = 0,
            slideTransfer = -5,
            slideAutoFar = 18.5,
            slideAutoClose = 14;
    private final double slideThreshold = 8;
    double distance;

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
        READY(0.53),
        TRANSFER_BACK(0.48),
        TRANSFER_FRONT(0.58),
        HALF_TRANSFER(0.8),
        AURA_TRANSFER(0.52),
        TRANSFER_META(0.68),
        TRANSFER_FINISH(0.55),
        HORIZONTAL(0.82),
        HP_DEPOSIT(0.3),
        HALF_DOWN(0.9),
        AURA_DOWN(0.93),
        DOWN(1),
        EXTENDO_DOWN(0.97), // this exists because the intake saggs on the slides
        BACK(0.98),
        IN(0.45),
        AROUND(0.5);

        public final double pos;

        IntakeArmServoState(double pos)
        {
            this.pos = pos;
        }
    }

    public enum IntakeClipServoState
    {
        HOLD(0.52),
        OPEN(0.2);

        public final double pos;

        IntakeClipServoState(double pos)
        {
            this.pos = pos;
        }

    }

    public enum IntakeTurretServoState
    {
        TRANSFER_META(0.797),
        MAX_LEFT(0.53),
        LEFT(0.45),
        STRAIGHT(0.37),
        RIGHT(0.27),
        MAX_RIGHT(0.21),
        HP_DEPOSIT(0.62),
        AROUND(0.94);

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
    public enum SampleColour
    {
        RED,
        BLUE,
        YELLOW
    }



    public IntakeFilter intakeFilter = IntakeFilter.NEUTRAL;

    private final double TICKS_PER_BAREMOTOR = 28;
    public boolean isRed = false, isYellow = false;

    public IntakeSubsystem(GeneralHardware hardware)
    {
        side = hardware.side;

        turretS = hardware.turretS;
        intakeCS = hardware.intakeCS;
        clipS = hardware.clipS;
        armS = hardware.intakeArmS;
        colourSensor = hardware.colourSensor;
        colourSupplier = new TimedSupplier<>(() -> colourSensor.alpha(), 40);
        distanceSensor = hardware.distanceSensor;
        distanceSupplier = new TimedSupplier<>(() -> distanceSensor.getDistance(DistanceUnit.INCH), 40);
        intakeSlideMotor = hardware.intakeSlidesM;

        intakeHardwareSetUp(); // this can now be called from here because the objects initialize at hardware
    }
    public IntakeSubsystem(HardwareMap hardware)
    {
        side = GeneralHardware.Side.RED;

        turretS = new ServoPika(hardware.get(ServoImplEx.class, "turretS"));
        armS = new ServoPika(hardware.get(ServoImplEx.class, "armS"));
        intakeCS = new CRServoPika(hardware.get(CRServoImplEx.class, "intakeCS"));
        clipS = new ServoPika(hardware.get(ServoImplEx.class, "clipS"));
        colourSensor = hardware.get(RevColorSensorV3.class, "colourSensor"); // no supplier as i want this to pool immediately and synchronously
        distanceSensor = hardware.get(DistanceSensor.class, "distanceSensor");
        intakeSlideMotor = new MotorPika(hardware.get(DcMotorEx.class, "intakeSlides"));

        intakeHardwareSetUp(); // this can now be called from here because the objects initialize at hardware
    }


    public void intakeHardwareSetUp()
    {
        // if we need to reverse anything
        intakeCS.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void intakeReads(boolean i2c)
    {
        intakeReads(i2c, false);
    }
    public void intakeReads(boolean i2c, boolean auto)
    {
        slidePosition = intakeSlideMotor.getCurrentPosition();
        if (i2c)
        {
            colorValue = colourSupplier.get();
            colourSensorDistance = colourSensor.getDistance(DistanceUnit.INCH);
            if (auto) distance = distanceSupplier.get();
        }
    }

    public void intakeSpin(IntakeSpinState state)
    {
        switch (state)
        {
            case INTAKE:
                intakeCS.setPower(1);
                break;
            case OFF:
                intakeCS.setPower(0);
                break;
            case REVERSE:
                intakeCS.setPower(-1);
                break;
        }
    }

    public void intakeSpin(double spinDirection)
    {
        intakeCS.setPower(spinDirection);
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
        turretS.setPosition(angle * 0.002816901408 + 0.075); // 1 / 355, +0.075 offset to zero
    }

    public void intakeTurretBasedOnHeadingVel(double headingVel)
    {
        double t = Math.min(1, Math.abs(headingVel / 100));
        t = Math.pow(t, 1.5); // this makes it only start moving at around 40%
        double i = interpolation(0, 60, t);
        turretS.setPosition(STRAIGHT.pos + angleToServoTicks(i) * Math.signum(headingVel));
    }

    public double angleToServoTicks(double angle)
    {
        return angle * (1 / 355.0);
    }

    private double interpolation(double p1, double p2, double t)
    {
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
        if (slideTarget > slideExtensionLimit)
            slideTarget = (int) inchesToTicksSlidesMotor(slideExtensionLimit);
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
        intakeSlideMotor.setPower(manualControlIntakeSlide);
    }
    public double distance(double r1, double g1, double b1, double r2, double g2, double b2) {
        return Math.sqrt((r1 - r2)*(r1 - r2) + (b1 - b2)*(b1 - b2) + (g1 - g2)*(g1 - g2));
    }

    public boolean checkColour(IntakeFilter filter) {

//        double red = colourSensor.red();
//        double green = colourSensor.green();
//        double blue = colourSensor.blue();
//
//        double redError = distance(red, green, blue, 255, 0, 0);
//        double yellowError = distance(red, green, blue, 255, 255, 0);
//        double blueError = distance(red, green, blue, 0, 0, 255);
//
//        if (redError < yellowError && redError < blueError) return "red";
//        if (yellowError < redError && yellowError < blueError) return "yellow";
//        if(blueError < redError && blueError < yellowError)return "blue";
//        return "null";

        final float[] hsvValues = new float[3];

        android.graphics.Color.colorToHSV(colourSensor.getNormalizedColors().toColor(), hsvValues);
        SampleColour colourOfSample;
        if(hsvValues[0] < 60 )
        {
            colourOfSample = SampleColour.RED;
        } else if(hsvValues[0] > 120)
        {
            colourOfSample = SampleColour.BLUE;
        } else colourOfSample = SampleColour.YELLOW;
        switch (filter)
        {
            case SIDE_ONLY:
                return side == GeneralHardware.Side.RED ? colourOfSample == SampleColour.RED : colourOfSample == SampleColour.BLUE;
            case NEUTRAL:
                return colourOfSample == SampleColour.YELLOW ||
                        side == GeneralHardware.Side.RED ? colourOfSample == SampleColour.RED : colourOfSample == SampleColour.BLUE;
            case YELLOW_ONLY:
                return colourOfSample == SampleColour.YELLOW;
            default: // catches filter == off
                return true;
        }
    }
    public boolean colorLogic()
    {
        switch (intakeFilter)
        {
            case SIDE_ONLY:
                if (isYellow) return false;
                else
                    return isRed ? side == GeneralHardware.Side.RED : side == GeneralHardware.Side.BLUE;
            case NEUTRAL:
                return (isYellow) ||
                        isRed ? side == GeneralHardware.Side.RED : side == GeneralHardware.Side.BLUE;
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
    public boolean isDistance(double inch)
    {
        return distance < inch;
    }
    public double getDistance()
    {
        return distance;
    }
    public double getColourSensorDistance()
    {
        return colourSensorDistance;
    }
    public double armLength(double armAngle)
    {
        double d = 6.5;
        return d / Math.cos(Math.toRadians(Math.abs(armAngle)));
    }
    public Vector armPosIn2dPlane(double armAngle)
    {
        double y =  6.5 / Math.cos(Math.toRadians(Math.abs(armAngle)));
        double x = 6.5 / Math.sin(Math.toRadians(Math.abs(armAngle)));
        return new Vector(x, y);
    }
    public void intakeInverseKinematics(Pose robot, Pose sample)
    {
        double armAngle = sample.getHeading() * -1;
        double trueArmAngle = armAngle - robot.getHeading();

        double armYOffSet = 6.5 * Math.cos(trueArmAngle);
        double armXOffSet = 6.5 * Math.sin(trueArmAngle);
        Pose extendoPose = sample.plus(new Pose(armXOffSet, armYOffSet));
        double intakeDis = extendoPose.getDistance(robot);

        intakeSlideInternalPID(intakeDis);
        intakeTurretSetAngle(trueArmAngle);
    }
    public boolean isSample()
    {
        if (isYellow) return true;
        else return isRed ? side == GeneralHardware.Side.RED : side == GeneralHardware.Side.BLUE;
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
        return slidePosition < 8; // this works as slide base is 0
    }

    public double ticksToInchesSlidesMotor(double ticks){
        return 24 * (ticks / 611);
        //return ((1.005007874 * 2 * Math.PI) / (TICKS_PER_BAREMOTOR * 5.6428571429)) * ticks;
    }

    public double inchesToTicksSlidesMotor (double inches){
        return 611 * (inches / 24);
        //return ((TICKS_PER_BAREMOTOR * 5.6428571429)/(1.005007874 * 2 * Math.PI)) * inches; //ticks per inches
        // ratio is 70/12 = 5
    }

}
