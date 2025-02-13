package org.firstinspires.ftc.teamcode.system.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.system.accessory.math.Angles;
import org.firstinspires.ftc.teamcode.system.accessory.pids.PID;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;
import org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers.MotorPika;
import org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers.ServoPika;

import java.util.HashMap;
import java.util.Map;

public class OuttakeSubsystem
{

    ServoPika clawS, wristS, pivotS, armS;

    MotorPika liftMotor, turretMotor;
    PID liftPID = new PID(0.04, 0, 0.003, 0, 0);
    PID turretPID = new PID(0.04, 0, 0.003, 0, 0);

    public int liftTarget, liftPosition;
    public double turretTarget, turretAngle;

    public int turretPosition;
    GeneralHardware.Side side;
    private final double TICKS_PER_BARE_MOTOR = 28;

    public static final double
            liftMaxExtension = 14, // 1655ticks
            liftHighBucketPos = 14,
            liftLowBucketPos = 2,
            liftHighBarPos = 6,
            liftLowBarPos = 0,
            liftSpecimenIntakePos = 0,
            liftBasePos = 0; // maybe make this -5 because of shit intake clip
    private final double liftThreshold = 8;
    public static final double // degree
            turretFrontAngle = 0,
            turretBackAngle = 180;
    public final double turretThreshold = 2;
    private final double maxAngleAxon = 355;

    public enum OuttakePivotServoState
    {
        UP(4, 0),
        RIGHT_UP(5, 0),
        RIGHT(6, 0),
        RIGHT_DOWN(0, 0),
        DOWN(0, 0),
        LEFT_DOWN(1, 0),
        LEFT(2, 0),
        LEFT_UP(3, 0);

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
        OPEN(0),
        CLOSE(0),
        INTAKE(0);

        public final double pos;

        OuttakeClawServoState(double pos)
        {
            this.pos = pos;
        }
    }

    public enum OuttakeArmServoState
    {

        READY(0),
        SPIN(0),
        STRAIGHT(0),
        TRANSFER_FRONT(0),
        TRANSFER_BACK(0),
        HALF_TRANSFER(0),
        SAMPLE(0),
        SPECIMEN_HIGH(0),
        SPECIMEN_HIGH_BACK(0),
        SPECIMEN_LOW(0),
        SPECIMEN_LOW_BACK(0),
        INTAKE(0),
        HP_DEPOSIT(0);

        public final double pos;

        OuttakeArmServoState(double pos)
        {
            this.pos = pos;
        }
    }

    public enum OuttakeWristServoState
    {
        READY(0),
        SPIN(0),
        TRANSFER_FRONT(0),
        TRANSFER_BACK(0),
        TRANSFER_FINISH(0),
        SAMPLE(0),
        SAMPLE_DROP(0),
        SPECIMEN_HIGH(0),
        SPECIMEN_HIGH_BACK(0),
        SPECIMEN_LOW(0),
        SPECIMEN_LOW_BACK(0),
        HP_DEPOSIT(0),
        INTAKE(0);

        public final double pos;

        OuttakeWristServoState(double pos)
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

        liftMotor = hardware.outtakeLiftM;
        outtakeHardwareSetUp(); // this can now be called from here because the objects initialize at hardware
    }

    public void outtakeHardwareSetUp()
    {
        // if we need to reverse anything
        // i want this to be done inside the hardware class
        //clawS.setDirection(Servo.Direction.REVERSE);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void outtakeReads()
    {
        liftPosition = liftMotor.getCurrentPosition();
        turretPosition = turretMotor.getCurrentPosition();
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
    public void pivotSerPosByOrder(int order)
    {
        pivotS.setPosition(OuttakePivotServoState.valueOfOrder(order).pos);
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

    public void turretSpinTo(double turretTargetAngle)
    {
        turretTarget = Angles.normalizeDegrees(turretTargetAngle - turretAngle);
        double power = turretPID.update(turretTarget, 0, 1);
        turretMotor.setPower(power);
    }
    public void turretSpinTo(double turretTargetAngle, double maxPower)
    {
        turretTarget = Angles.normalizeDegrees(turretTargetAngle - turretAngle);
        double power = turretPID.update(turretTarget, 0, maxPower);
        turretMotor.setPower(power);
    }
    public void turretKeepToAngle(double turretTargetAngle, double heading, boolean back) // the target angle is field centric ig
    {
        turretTargetAngle += back ? 180 : 0;
        double diff = Angles.normalizeDegrees(turretTargetAngle - heading);
        turretTarget = Angles.normalizeDegrees(diff - turretAngle);
        double power = turretPID.update(turretTarget, 0, 1);
        turretMotor.setPower(power);
    }
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
        return liftPosition < 20;
    }

    public double ticksToInchesSlidesMotor(double ticks)
    {
        return 0.01276363636 * ticks;
        // 0.58 is the radius of the pulley
        // and 1.2 is the ratios of teeth
    }

    public double inchesToTicksSlidesMotor(double inches)
    {
        return 78.3475783476 * inches;
    }

    public boolean isArmOver()
    {
        return armS.getPosition() > OuttakeArmServoState.STRAIGHT.pos;
    }

    public double getArmAngle()
    {
        return servoPosToAngle(armS.getPosition());
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
