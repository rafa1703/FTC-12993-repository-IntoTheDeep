package org.firstinspires.ftc.teamcode.system.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.gvf.utils.Encoder;
import org.firstinspires.ftc.teamcode.gvf.utils.LowPassFilter;
import org.firstinspires.ftc.teamcode.system.accessory.math.Angles;
import org.firstinspires.ftc.teamcode.system.accessory.pids.PID;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;
import org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers.MotorPika;
import org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers.ServoPika;

import java.util.HashMap;
import java.util.Map;

public class OuttakeSubsystem
{

    ServoPika clawS, wristS, pivotS, armS, lockS;

    MotorPika liftMotor, turretMotor;
    Encoder turretIncrementalEncoder;
    AnalogInput turretEncoder;
    PID liftPID = new PID(0.04, 0, 0.003, 0, 0);
    PID turretPID = new PID(0.0013, 0.01, 0.00002, 35, 0);
    PID turretAbsolutePID = new PID(0.012, 0.03, 0.0008, 10, 0);
    LowPassFilter turretFilter = new LowPassFilter(0.8, 0);

    public int liftTarget, liftPosition;
    public double turretTarget, turretAngle;

    GeneralHardware.Side side;
    private final double TICKS_PER_BARE_MOTOR = 28;

    public double
            liftMaxExtension = 31.7,
            liftHighBucketPos = 22.5,
            liftLowBucketPos = 9 + 2,
            liftHighBarPos = 13.5 + 2,
            liftHighBarBackStaticPos = 11,
            liftHighBarBackAutoPos = 10 + 2,
            liftHighBarBackKineticPos = 17 + 2,
            liftLowBarPos = 0,
            liftSpecimenIntakePos = 0,
            liftBasePos = 0;
    private final double liftThreshold = 8;
    public static final double // degree
            turretFrontAngle = 0,
            turretBackAngle = 180;
    public final double turretThreshold = 0.75; // degrees, this is kinda low but the turret position
    private final double maxAngleAxon = 355;
    public int turretIncrementalPosition;
    public double initialOffsetPosition;

    public double getPositionIn()
    {
        return ticksToInchesSlidesMotor(liftPosition);
    }

    public enum OuttakeTurretState
    {
        TRANSFER_FRONT(0),
        TRANSFER_BACK(180),
        TRANSFER_META(142),
        HP_DROP_AUTO(-90),
        CLIMB_START(150),
        SPEC_DEPOSIT_BACK(252);

        public final double angle;
        OuttakeTurretState(double angle)
        {
            this.angle = angle;
        }
    }
    public enum OuttakePivotServoState
    {
        UP(2, 0.13),
        RIGHT_UP(1, 1),
        RIGHT(0, 0.97),
        RIGHT_DOWN(7, 0.83),
        DOWN(6, 0.69),
        LEFT_DOWN(5, 0.58),
        LEFT(4, 0.415),
        LEFT_UP(3, 0.3);

        private static final Map<Double, OuttakePivotServoState> BY_POSITION = new HashMap<>();
        private static final Map<Integer, OuttakePivotServoState> BY_ORDER = new HashMap<>();

        static
        {
            for (OuttakePivotServoState e : values())
            {
                BY_ORDER.put(e.order, e);
                BY_POSITION.put(e.pos, e);
            }
        }

        public final double pos;
        public final int order;

        OuttakePivotServoState(int order, double pos)
        {
            this.pos = pos;
            this.order = order;
        }

        public static OuttakePivotServoState valueOfPosition(double number)
        {
            return BY_POSITION.get(number);
        }
        public static OuttakePivotServoState valueOfOrder(int order)
        {
            return BY_ORDER.get(order);
        }

    }
    public enum OuttakeClawServoState
    {
        OPEN(0.88),
        CLOSE(0.52),
        INTAKE(0.94),
        TRANSFER_FRONT(0.99);

        public final double pos;

        OuttakeClawServoState(double pos)
        {
            this.pos = pos;
        }
    }

    public enum OuttakeArmServoState
    {

        READY(0.1),
        SPIN(0.45),
        STRAIGHT(0.51), // -0.1
        TRANSFER_FRONT(0.845),
        TRANSFER_BACK(0.215), // new thing so the huge fucking claw doesn't hit the fat servo (as fat as our cad designer)
        TRANSFER_AURA(0.25),
        TRANSFER_META(0.1),
        SAMPLE(0.46),
        SPECIMEN_HIGH(0.9), // 0.7
        SPECIMEN_AUTO_PRELOADS(0.69),
        SPECIMEN_HIGH_AUTO_SCORE(0.67),
        SPECIMEN_HIGH_BACK_STATIC(0.46),
        SPECIMEN_HIGH_BACK_KINETIC(0.66),
        SPECIMEN_LOW(0.78),
        SPECIMEN_LOW_BACK(0.27),
        INTAKE(0),
        HP_DEPOSIT(0.01);

        public final double pos;

        OuttakeArmServoState(double pos)
        {
            this.pos = pos;
        }
    }

    public enum OuttakeWristServoState
    {
        READY(0),
        SPIN(0.55),
        TRANSFER_FRONT(0.59),
        TRANSFER_BACK(0.24), //-0.16
        TRANSFER_AURA(0.18),
        TRANSFER_META(0.37),
        SAMPLE(0.5), // 435
        SAMPLE_DROP(0.57),
        SPECIMEN_HIGH(0.18), // 0.3
        SPECIMEN_AUTO_PRELOADS(0.6),
        SPECIMEN_HIGH_FLICK(0.4),
        SPECIMEN_HIGH_AUTO(0.61), // 0.45 prev
        SPECIMEN_HIGH_BACK_STATIC(0.8),
        SPECIMEN_HIGH_BACK_KINETIC(0.44),
        SPECIMEN_HIGH_BACK_FLICK(0.64),
        HP_DEPOSIT(0.43),
        INTAKE(0.47);

        public final double pos;

        OuttakeWristServoState(double pos)
        {
            this.pos = pos;
        }
    }
    public enum OuttakeLockServoState
    {
        OPEN(0.2),
        LOCKED(0.45);

        public final double pos;
        OuttakeLockServoState(double pos)
        {
            this.pos = pos;
        }

    }



    public OuttakeSubsystem(GeneralHardware hardware)
    {
        side = hardware.side;

        wristS = hardware.wristS;
        clawS = hardware.clawS;
        pivotS = hardware.pivotS;
        armS = hardware.armS;
        lockS = hardware.lockS;

        liftMotor = hardware.outtakeLiftM;
        turretMotor = hardware.turretM;
        turretEncoder = hardware.turretEncoder;
        turretIncrementalEncoder = hardware.turretIncrementalEncoder;
        outtakeHardwareSetUp(); // this can now be called from here because the objects initialize at hardware
    }

    public void outtakeHardwareSetUp()
    {
        // if we need to reverse anything
        // i want this to be done inside the hardware class
        //clawS.setDirection(Servo.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        cacheTurretInitialPosition();
    }

    public void cacheTurretInitialPosition()
    {
        double angle = Angles.reduceDegrees((turretEncoder.getVoltage() / 3.225) * 360);
        initialOffsetPosition = turretAngleToTicks(angle); // turretAngleToTicks(turretAngle());
        if (angle >= 270 && angle <= 360) initialOffsetPosition = initialOffsetPosition - 4000;
    }
    public void resetTurretPosition()
    {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cacheTurretInitialPosition();
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void outtakeReads()
    {
        outtakeReads(true);
    }
    public void outtakeReads(boolean i2c)
    {
        liftPosition = liftMotor.getCurrentPosition();
        turretIncrementalPosition = turretMotor.getCurrentPosition() + (int) initialOffsetPosition; // + initialOffsetPosition;//turretIncrementalEncoder.getCurrentPosition();// - initialOffsetPosition;
        if (i2c) turretAngle = turretAngle();
    }
    /** returns the absolute angle **/
    public double turretAngle()
    {
        return turretFilter.getValue((turretEncoder.getVoltage() / 3.225) * 360);
    }
    public double encoderVoltage()
    {
        return turretEncoder.getVoltage();
    }

    public void pivotServoState(OuttakePivotServoState state)
    {
        pivotS.setPosition(state.pos);
    }
    public void pivotSetPos(double pos)
    {
        pivotS.setPosition(pos);
    }
    public double pivotGetPos()
    {
        return pivotS.getPosition();
    }
    public void pivotSetPosByAngle(double angle)
    {
        pivotS.setPosition(0.002816901408 * angle);
    }
    public void pivotSetPosByOrder(int order)
    {
        pivotS.setPosition(OuttakePivotServoState.valueOfOrder(order).pos);
    }
    public void lockServoState(OuttakeLockServoState state)
    {
        lockS.setPosition(state.pos);
    }
    public void lockSetPos(double pos)
    {
        lockS.setPosition(pos);
    }
    public void clawState(OuttakeClawServoState state)
    {
        clawS.setPosition(state.pos);
    }

    public void clawSetPos(double pos)
    {
        clawS.setPosition(pos);
    }

    public void armState(OuttakeArmServoState state)
    {
        armS.setPosition(state.pos);
    }

    public void armSetPos(double armPos)
    {
        armS.setPosition(armPos);
    }

    public void wristState(OuttakeWristServoState state)
    {
        wristS.setPosition(state.pos);
    }

    public void wristSetPos(double pos)
    {
        wristS.setPosition(pos);
    }

    public void turretSpinTo(OuttakeTurretState state)
    {
        turretTarget = state.angle;
        double pow = turretAbsolutePID.updateImprovedControllers(state.angle, turretTicksToAngle(turretIncrementalPosition), 1);
        turretMotor.setPower(-pow);
    }
    public void turretSpinToGains(OuttakeTurretState state)
    {
        turretTarget = state.angle;
        double pow = turretAbsolutePID.updateImprovedControllers(state.angle, turretTicksToAngle(turretIncrementalPosition), 1);
        turretMotor.setPower(-pow);
    }
    public void turretSpinToGains(double angle)
    {
        turretTarget = angle;
        double pow = turretAbsolutePID.updateImprovedControllers(angle, turretTicksToAngle(turretIncrementalPosition), 1);
        turretMotor.setPower(-pow);
    }
//    public void turretSpinToTicks(double angle)
//    {
//        turretTarget = angle;
//        double pow = turretPID.update(turretAngleToTicks(angle), turretIncrementalPosition, 1);
//        turretMotor.setPower(pow);
//    }
    @Deprecated
    public void turretSpinToCorrected(double turretTargetAngle)
    {
        turretTarget = Angles.normalizeDegrees(turretTargetAngle - turretAngle);
        double incrementalFinalAngle = turretTicksToAngle(turretIncrementalPosition) + turretTarget;
        // if the incremental pos is greater than the wiring limit we must turn the other way to fix it
        if (incrementalFinalAngle >= 400 || incrementalFinalAngle <= -400)
        {
            turretTarget += 360 * Math.signum(incrementalFinalAngle) * -1; // go the opposite way
        }
        double power = turretPID.update(turretTarget, 0, 1);
        turretMotor.setPower(power);
    }
    public double turretAngleToTicks(double angle)
    {
        return angle / 360 * 4000;
    }
    public double turretTicksToAngle(double ticks)
    {
        return ticks / 4000 * 360;
    }
    @Deprecated
    public void turretSpinTo(double turretTargetAngle)
    {
        turretTarget = Angles.normalizeDegrees(turretTargetAngle - turretAngle);
        double power = turretPID.update(turretTarget, 0, 1);
        turretMotor.setPower(power);
    }
    @Deprecated
    public void turretSpinTo(double turretTargetAngle, double maxPower)
    {
        turretTarget = Angles.normalizeDegrees(turretTargetAngle - turretAngle);
        double power = turretPID.update(turretTarget, 0, maxPower);
        turretMotor.setPower(power);
    }
    @Deprecated
    public void turretKeepToAngle(double turretTargetAngle, double heading, boolean back) // the target angle is field centric ig
    {
        turretTargetAngle += back ? 180 : 0;
        heading = -heading;
        double diff = Angles.normalizeDegrees(turretTargetAngle - heading);
        turretTarget = Angles.normalizeDegrees(diff - turretAngle);
        double power = turretAbsolutePID.update(turretTarget, 0, 1);
        turretMotor.setPower(power);
    }
    public void turretKeepToAngleTicks(double turretTargetAngle, double heading) // the target angle is field centric ig
    {
//         turretTargetAngle += back ? 180 : 0;
        double turretAngle = Angles.reduceDegrees(turretTicksToAngle(turretIncrementalPosition));
        heading = -heading;
        double diff = Angles.normalizeDegrees(turretTargetAngle - heading);
        turretTarget = Angles.normalizeDegrees(diff - turretAngle);
        double power = turretAbsolutePID.update(turretTarget, 0, 1);
        turretMotor.setPower(-power);
    }
    @Deprecated
    public void turretKeepToAngle(double turretTargetAngle, double heading) // the target angle is field centric ig
    {
        turretKeepToAngle(turretTargetAngle, heading, false);
    }
    public boolean turretReached()
    {
        return Math.abs(Angles.normalizeDegrees(turretTarget - turretAngle)) < turretThreshold;
    }
    public boolean turretReached(double turretTargetAngle)
    {
        return Math.abs(Angles.normalizeDegrees(turretTargetAngle - turretAngle)) < turretThreshold;
    }
    public boolean turretReached(double turretTargetAngle, double angleThreshold)
    {
        return Math.abs(Angles.normalizeDegrees(turretTargetAngle - turretAngle)) < angleThreshold;
    }
    public void turretRawControl(double power)
    {
        turretMotor.setPower(power);
    }

//    public double turretAngleToTicks(double angle)
//    {
//        return angle / 360 * 2000.0;
//    }

//    public double ticksToTurretAngle(double ticks)
//    {
//        return turretPosition / 2000.0 * 360; // number of ticks of a full rev,
//    }

    public void liftTo(int rotations)
    {
        liftTarget = rotations;
        liftMotor.setPower(liftPID.update(rotations, liftPosition, 1));
    }

    public void liftTo(double inches)
    {
        liftTarget = (int) inchesToTicksSlidesMotor(inches);
        liftMotor.setPower(liftPID.update(liftTarget, liftPosition, 1));
    }

    public void liftToInternalPID(double inches)
    {
        liftTarget = (int) inchesToTicksSlidesMotor(inches);
        liftMotor.setTargetPosition(liftTarget);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(1);
    }

    public void liftToInternalPID(double inches, double pow)
    {
        liftTarget = (int) inchesToTicksSlidesMotor(inches);
        liftMotor.setTargetPosition(liftTarget);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(pow);
    }

    public void liftToInternalPIDTicks(int rotations)
    {
        liftTarget = rotations;
        liftMotor.setTargetPosition(liftTarget);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(1);
    }

    public void liftMotorEncoderReset()
    {
        liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void liftMotorRawControl(double manualControlLift)
    {
        liftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setPower(manualControlLift);
    }

    public boolean liftReached(double slideTargetInches)
    {
        return Math.abs(inchesToTicksSlidesMotor(slideTargetInches) - liftPosition) < liftThreshold;
    }

    public boolean liftAtBase() // as base is 0
    {
        return liftPosition < 30;
    }

    public double ticksToInchesSlidesMotor(double ticks)
    {
        return ticks / 964 * 22.5;
        // 0.58 is the radius of the pulley
        // and 1.2 is the ratios of teeth
    }

    public double inchesToTicksSlidesMotor(double inches)
    {
        return inches / 22.5 * 964;
    }

    public boolean isArmOver()
    {
        return armS.getPosition() > OuttakeArmServoState.STRAIGHT.pos;
    }

    public double getArmPos()
    {
        return armS.getPosition();
    }

    /**
     * Degrees
     **/
    public double angleToServoPos(double angle)
    {
        return angle / maxAngleAxon;
    }

    /**
     * Degrees
     **/
    public double servoPosToAngle(double pos)
    {
        return pos * maxAngleAxon;
    }
}
