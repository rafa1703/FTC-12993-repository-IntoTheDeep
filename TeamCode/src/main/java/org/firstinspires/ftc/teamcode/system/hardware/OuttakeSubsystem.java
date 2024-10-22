package org.firstinspires.ftc.teamcode.system.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.system.accessory.pids.PID;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

public class OuttakeSubsystem
{
    ServoImplEx clawS, pivotS, leftArmS, rightArmS;

    DcMotorEx liftMotor;
    PID intakeSlidesPID = new PID(0.04, 0, 0.003, 0, 0);

    public int liftTarget, liftPosition;
    double intakeSpeed = 1;
    GeneralHardware.Side side;
    private final double TICKS_PER_BAREMOTOR = 28;

    public static final double
        leftArmReadyPos = 1,
        leftArmTransferPos = 1,
        leftArmSamplePos = 0.5,
        leftArmSpecimenPos = 0.7,
        leftArmSpecimenScorePos = 0.65,
        leftArmIntakePos = 0.7;

    public static final double
        rightArmReadyPos = 0,
        rightArmTransferPos = 0,
        rightArmSamplePos = 1,
        rightArmSpecimenPos = 1,
        rightArmSpecimenScorePos = 1,
        rightArmIntakePos = 1;
    public static final double
        clawOpenPos = 0,
        clawClosePos = 0.1;
    public static final double
        pivotReadyPos = 0,
        pivotTransferPos = 0.15,
        pivotSamplePos = 0.3,
        pivotSpecimenPos = 0.2,
        pivotIntakePos = 0.28;

    public static final int
        liftHighBucketPos= 400,
        liftLowBucketPos = 300,
        liftHighBarPos = 200,
        liftLowBarPos = 100,
        liftSpecimenIntake = 150,
        liftBasePos = 0;
    private final double liftThreshold = 10;
    public enum OuttakeClawServoState
    {
        OPEN,
        CLOSE
    }
    public enum OuttakeArmServoState
    {
        READY,
        TRANSFER,
        SAMPLE,
        SPECIMEN,
        SPECIMEN_SCORE,
        INTAKE
    }
    public enum OuttakePivotServoState
    {
        READY,
        TRANSFER,
        SAMPLE,
        SPECIMEN,
        INTAKE
    }

    public OuttakeSubsystem(GeneralHardware hardware)
    {
        side = hardware.side;

        pivotS = hardware.pivotS;
        clawS = hardware.clawS;
        leftArmS = hardware.outtakeLeftArmS;
        rightArmS = hardware.outtakeRightArmS;

        liftMotor = hardware.outtakeLiftM;
        outtakeHardwareSetUp(); // this can now be called from here because the objects initialize at hardware
    }
    public OuttakeSubsystem(HardwareMap hardware, GeneralHardware.Side side)
    {
        this.side = side;

        pivotS = hardware.get(ServoImplEx.class, "pivotS");
        clawS = hardware.get(ServoImplEx.class, "clawS");
        leftArmS = hardware.get(ServoImplEx.class, "outtakeLeftArmS");
        rightArmS = hardware.get(ServoImplEx.class, "outtakeRightArmS");

        liftMotor = hardware.get(DcMotorEx.class, "OuttakeSlides");
        outtakeHardwareSetUp(); // this can now be called from here because the objects initialize at hardware
    }

    public void outtakeHardwareSetUp()
    {
        // if we need to reverse anything
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
                leftArmS.setPosition(leftArmReadyPos);
                //rightArmS.setPosition(rightArmReadyPos);
                break;
            case TRANSFER:
                leftArmS.setPosition(leftArmTransferPos);
                //rightArmS.setPosition(rightArmTransferPos);
                break;
            case SAMPLE:
                leftArmS.setPosition(leftArmSamplePos);
                //rightArmS.setPosition(rightArmSamplePos);
                break;
            case SPECIMEN:
                leftArmS.setPosition(leftArmSpecimenPos);
                //rightArmS.setPosition(rightArmSpecimenPos);
                break;
            case SPECIMEN_SCORE:
                leftArmS.setPosition(leftArmSpecimenScorePos);
                //rightArmS.setPosition(rightArmSpecimenScorePos);
                break;
            case INTAKE:
                leftArmS.setPosition(leftArmIntakePos);
                //rightArmS.setPosition(rightArmIntakePos);
                break;
        }
    }
    public void armSetPos(double leftArm, double rightArm)
    {
        leftArmS.setPosition(leftArm);
       // rightArmS.setPosition(rightArm);
    }

    public void pivotState(OuttakePivotServoState state)
    {
        switch (state)
        {
            case READY:
                pivotS.setPosition(pivotReadyPos);
                break;
            case TRANSFER:
                pivotS.setPosition(pivotTransferPos);
                break;
            case SAMPLE:
                pivotS.setPosition(pivotSamplePos);
                break;
            case SPECIMEN:
                pivotS.setPosition(pivotSpecimenPos);
                break;
            case INTAKE:
                pivotS.setPosition(pivotIntakePos);
                break;
        }
    }
    public void pivotSetPos(double pos)
    {
        pivotS.setPosition(pos);
    }

    public void liftTo(int rotations)
    {
        liftTarget = rotations;
        liftMotor.setPower(intakeSlidesPID.update(rotations, liftPosition, 1));
    }
    public void liftToInternalPID(double inches)
    {
        liftTarget = (int) inchesToTicksSlidesMotor(inches);
        liftMotor.setTargetPosition(liftTarget);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void liftToInternalPIDTicks(int rotations)
    {
        liftTarget = rotations;
        liftMotor.setTargetPosition(liftTarget);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void liftMotorEncoderReset()
    {
        liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void intakeSlideMotorRawControl(double manualControlIntakeSlide)
    {
        liftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setPower(manualControlIntakeSlide * 0.75);
    }
    public boolean liftReached(int slideTarget)
    {
        return Math.abs(slideTarget - liftTarget) < liftThreshold;
    }

    public double ticksToInchesSlidesMotor(double ticks){
        return ((1.005007874 * 2 * Math.PI) / (TICKS_PER_BAREMOTOR * 5.6428571429)) * ticks;
    }

    public double inchesToTicksSlidesMotor (double inches){
        return ((TICKS_PER_BAREMOTOR * 5) / (0.89221 * 2 * Math.PI)) * inches; //ticks per inches
        // ratio is 70/14 = 5
    }


}
