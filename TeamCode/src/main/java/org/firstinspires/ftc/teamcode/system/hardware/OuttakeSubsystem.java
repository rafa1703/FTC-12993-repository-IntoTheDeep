package org.firstinspires.ftc.teamcode.system.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

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

    public static final double
            armStraightPos = 0.55,
            armReadyPos = 0.2,
            armTransferPos = 0.14,
            armTransferFinishPos = 0.1,
            armSamplePos = 0.58,
            armSpecimenPos = 0.35,
            armIntakePos = 0.95;
    public static final double
            clawOpenPos = 0.8,
            clawIntakePos = 0.92,
            clawClosePos = 0.5;
    public static final double
            wristReadyPos = 0.35,
            wristTransferPos = 0.415,
            wristTransferFinishPos = 0.24,
            wristPerpendicularPos = 0.34,
            wristSamplePos = 0.67,
            wristSampleDropPos = 0.76,
            wristSpecimenPos = 0.4,
            wristIntakePos = 0.55;
    public static final double
            railReadyPos = 0.45,
            railHighPos = 0.135,
            railMiddlePos = 0.55,
            railOverTheTopPos = 0.35,
            railTransferPos = 0.889,
            railTransferFinishPos = 0.75,
            railSpecimenLowPos = 0.83,
            railSpecimenHighPos = 0.3,
            railSamplePos = railHighPos,
            railIntakePos = 0.49,
            railLowPos = 0.95;

    public static final double
            liftMaxExtension = 14, // 1655ticks
            liftHighBucketPos= 14,
            liftLowBucketPos = 2,
            liftHighBarPos = 6,
            liftLowBarPos = 0,
            liftSpecimenIntake = 0,
            liftBasePos = 0; // maybe make this -5 because of shit intake clip
    private final double liftThreshold = 8;
    private final double maxAngleAxon = 255;
    private final double armFFCoeff = 0.1;
    private double outtakeRailAdjustTimer;

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
        SAMPLE_DROP,
        SPECIMEN,
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
        railS = hardware.railS;
        armS = hardware.armS;

        liftMotor = hardware.outtakeLiftM;
        outtakeHardwareSetUp(); // this can now be called from here because the objects initialize at hardware
    }
    public void outtakeHardwareSetUp()
    {
        // if we need to reverse anything
        // i want this to be done inside the hardware class
        //clawS.setDirection(Servo.Direction.REVERSE);
//        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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
            case SAMPLE_DROP:
                wristS.setPosition(wristSampleDropPos);
                break;
            case SPECIMEN:
                wristS.setPosition(wristSpecimenPos);
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
    public double railGetPos()
    {
        return railS.getPosition();
    }
    public void fineAdjustRail(double fineAdjust, double timer){ // this is for tinkos controls only
        double currentRailPos = railS.getPosition();
        if (timer - outtakeRailAdjustTimer > 15){ // only set the position every 15 ms, once achieved cache the timer value
            currentRailPos += fineAdjust * 0.03; // changes global variale at .05 per 15ms
            if (currentRailPos > railLowPos){ // rail low is 0.95
                currentRailPos = railLowPos;
            } else if (currentRailPos < railHighPos){ // rail high is 0.17
                currentRailPos = railHighPos;
            }
            railS.setPosition(currentRailPos);
            outtakeRailAdjustTimer = timer; // cache the value of the outtakerailadjust
        }
        // this should make the fine adjust not looptime dependent. can tune by adjusting iteration & move amount
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
        return liftPosition < 20;
    }
    //TODO add new measurements, and remember this is not a bare motor anymore
    public double ticksToInchesSlidesMotor(double ticks){
        return 0.01276363636 * ticks;
        // 0.58 is the radius of the pulley
        // and 1.2 is the ratios of teeth
    }

    public double inchesToTicksSlidesMotor (double inches){
        return 78.3475783476 * inches;
    }

    public boolean isArmOver()
    {
        return armS.getPosition() > armStraightPos;
    }
    public boolean isRailUnderTheTop()
    {
        return railS.getPosition() < railOverTheTopPos;
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
