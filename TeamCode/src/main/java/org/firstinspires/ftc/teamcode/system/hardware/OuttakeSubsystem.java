package org.firstinspires.ftc.teamcode.system.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.system.accessory.pids.PID;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;
import org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers.MotorPika;
import org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers.ServoPika;

public class OuttakeSubsystem
{

    ServoPika clawS, wristS, railS, armS;

    MotorPika liftMotor;
    PID liftPID = new PID(0.04, 0, 0.003, 0, 0);

    public int liftTarget, liftPosition;
    GeneralHardware.Side side;
    private final double TICKS_PER_BARE_MOTOR = 28;

    public static final double // make sure straight is zero, so over the top is 180º
            armStraightPos = 0.5,
            armReadyPos = 0.9,
            armTransferPos = 0.885,
            armTransferFinishPos = 0.8,
            armSamplePos = 0.4,
            armSpecimenPos = 0.72,
            armIntakePos = 0.775;
    public static final double
            clawOpenPos = 0.3,
            clawIntakePos = 0.1,
            clawClosePos = 0.665;
    public static final double
            wristReadyPos = 0.68,
            wristTransferPos = 0.53,
            wristTransferFinishPos = 0.65,
            wristPerpendicularPos = 0.5,
            wristSamplePos = 0.585,
            wristSpecimenPos = 0.31,
            wristSpecimenDropPos = 0.5,
            wristIntakePos = 0.55;
    public static final double
            railReadyPos = 0,
            railHighPos = 0,
            railMiddlePos = 0,
            railOverTheTopPos = 0,
            railTransferPos = 1,
            railTransferFinishPos = 1,
            railSpecimenLowPos = 0,
            railSpecimenHighPos = 0,
            railSamplePos = 0,
            railIntakePos = 0,
            railLowPos = 1;

    public static final double
            liftMaxExtension = 27, // 680 ticks
            liftHighBucketPos= 24,
            liftLowBucketPos = 1,
            liftHighBarPos = 19,
            liftLowBarPos = 4,
            liftSpecimenIntake = 1,
            liftBasePos = 0; // maybe make this -5 because of shit intake clip
    private final double liftThreshold = 10;
    private final double maxAngleAxon = 255;
    private final double armFFCoeff = 0.1;

    public enum OuttakeClawServoState
    {
        OPEN,
        INTAKE,
        CLOSE
    }
    public enum OuttakeArmServoState
    {
        READY,
        STRAIGHT,
        TRANSFER,
        TRANSFER_FINISH,
        SAMPLE,
        SPECIMEN,
        INTAKE
    }
    public enum OuttakeWristServoState
    {
        READY,
        TRANSFER,
        TRANSFER_FINISH,
        PERPENDICULAR,
        SAMPLE,
        SPECIMEN,
        SPECIMEN_DROP,
        INTAKE
    }
    public enum OuttakeRailServoState
    {
        READY,
        HIGH,
        MIDDLE,
        LOW,
        OVER_THE_TOP,
        TRANSFER,
        TRANSFER_FINISH,
        SAMPLE,
        SPECIMEN_LOW,
        SPECIMEN_HIGH,
        INTAKE
    }

    public OuttakeSubsystem(GeneralHardware hardware)
    {
        side = hardware.side;

        wristS = hardware.wristS;
        clawS = hardware.clawS;
        railS = hardware.outtakeLeftArmS;
        armS = hardware.outtakeRightArmS;

        liftMotor = hardware.outtakeLiftM;
        outtakeHardwareSetUp(); // this can now be called from here because the objects initialize at hardware
    }
    public void outtakeHardwareSetUp()
    {
        // if we need to reverse anything
        // i want this to be done inside the hardware class
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void outtakeReads()
    {
        liftPosition = liftMotor.getCurrentPosition();
    }

    public void clawState(OuttakeClawServoState state)
    {
        switch (state)
        {
            case OPEN:
                clawS.setPosition(clawOpenPos);
                break;
            case INTAKE:
                clawS.setPosition(clawIntakePos);
                break;
            case CLOSE:
                clawS.setPosition(clawClosePos);
                break;
        }
    }
    public void clawSetPos(double pos)
    {
        clawS.setPosition(pos);
    }

    public void armState(OuttakeArmServoState state)
    {
        switch (state)
        {
            case READY:
                armS.setPosition(armReadyPos);
                break;
            case STRAIGHT:
                armS.setPosition(armStraightPos);
                break;
            case TRANSFER:
                armS.setPosition(armTransferPos);
                break;
            case TRANSFER_FINISH:
                armS.setPosition(armTransferFinishPos);
                break;
            case SAMPLE:
                armS.setPosition(armSamplePos);
                break;
            case SPECIMEN:
                armS.setPosition(armSpecimenPos);
                break;
            case INTAKE:
                armS.setPosition(armIntakePos);
                break;
        }
    }
    public void armSetPos(double armPos)
    {
        armS.setPosition(armPos);
    }

    public void wristState(OuttakeWristServoState state)
    {
        switch (state)
        {
            case READY:
                wristS.setPosition(wristReadyPos);
                break;
            case TRANSFER:
                wristS.setPosition(wristTransferPos);
                break;
            case TRANSFER_FINISH:
                wristS.setPosition(wristTransferFinishPos);
                break;
            case PERPENDICULAR:
                wristS.setPosition(wristPerpendicularPos);
                break;
            case SAMPLE:
                wristS.setPosition(wristSamplePos);
                break;
            case SPECIMEN:
                wristS.setPosition(wristSpecimenPos);
                break;
            case SPECIMEN_DROP:
                wristS.setPosition(wristSpecimenDropPos);
                break;
            case INTAKE:
                wristS.setPosition(wristIntakePos);
                break;
        }
    }
    public void wristSetPos(double pos)
    {
        wristS.setPosition(pos);
    }

    public void railState(OuttakeRailServoState state)
    {
        switch (state)
        {
            case READY:
                railS.setPosition(railReadyPos);
                break;
            case HIGH:
                railS.setPosition(railHighPos);
                break;
            case MIDDLE:
                railS.setPosition(railMiddlePos);
                break;
            case LOW:
                railS.setPosition(railLowPos);
                break;
            case OVER_THE_TOP:
                railS.setPosition(railOverTheTopPos);
                break;
            case TRANSFER:
                railS.setPosition(railTransferPos);
                break;
            case TRANSFER_FINISH:
                railS.setPosition(railTransferFinishPos);
                break;
            case SAMPLE:
                railS.setPosition(railSamplePos);
                break;
            case SPECIMEN_LOW:
                railS.setPosition(railSpecimenLowPos);
                break;
            case SPECIMEN_HIGH:
                railS.setPosition(railSpecimenHighPos);
                break;
            case INTAKE:
                railS.setPosition(railIntakePos);
                break;
        }
    }
    public void railSetPos(double railPos)
    {
        railS.setPosition(railPos);
    }

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
    public void liftWithFeedForward(double inches)
    {

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
        return liftPosition < liftThreshold;
    }

    public double ticksToInchesSlidesMotor(double ticks){
        return ((0.95485 * 2 * Math.PI) / (TICKS_PER_BARE_MOTOR * 5.3571428571)) * ticks;
        // 0.9 is the radius of the pulley
        // and 5.35 is the ratios of teeth
    }

    public double inchesToTicksSlidesMotor (double inches){
        return ((TICKS_PER_BARE_MOTOR * 5.3571428571) / (0.95485 * 2 * Math.PI)) * inches;
    }
    public double getArmAngle()
    {
        return servoPosToAngle(armS.getPosition());
    }
    /**Degrees**/
    public double angleToServoPos(double angle)
    {
        return angle / maxAngleAxon;
    }
    /**Degrees**/
    public double servoPosToAngle(double pos)
    {
        return pos * maxAngleAxon;
    }
    public double armFeedForward(double angleDeg)
    {
        return armFFCoeff * Math.cos(Math.toRadians(angleDeg));
    }
    public double liftFeedForward()
    {
        double a = (liftPosition / inchesToTicksSlidesMotor(liftMaxExtension)) * Math.PI;
        return Math.pow(Math.cos(a), 8);
    }

}
