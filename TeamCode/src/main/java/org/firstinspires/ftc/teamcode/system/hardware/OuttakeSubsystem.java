package org.firstinspires.ftc.teamcode.system.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.system.accessory.pids.PID;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

public class OuttakeSubsystem
{
    ServoImplEx clawS, pivotS, leftArmS, rightArmS;

    DcMotorEx liftMotor;
    PID intakeSlidesPID = new PID(0.04, 0, 0.003, 0, 0);

    int liftTarget, liftPosition;
    double intakeSpeed = 1;
    GeneralHardware.Side side;

    public static final double
        leftArmReadyPos = 0,
        leftArmTransferPos = 0,
        leftArmSamplePos = 1,
        leftArmSpecimenPos = 1,
        leftArmSpecimenScorePos = 1,
        leftArmIntakePos = 1;

    public static final double
        rightArmReadyPos = 0,
        rightArmTransferPos = 0,
        rightArmSamplePos = 1,
        rightArmSpecimenPos = 1,
        rightArmSpecimenScorePos = 1,
        rightArmIntakePos = 1;
    public static final double
        clawOpenPos = 1,
        clawClosePos = 0;
    public static final double
        pivotReadyPos = 0,
        pivotTransferPos = 1,
        pivotSamplePos = 0.5,
        pivotSpecimenPos = 0,
        pivotIntakePos = 0;

    public static final int
        liftHighBucketPos= 400,
        liftLowBucketPos = 300,
        liftHighBarPos = 200,
        liftLowBarPos = 100,
        liftSpecimenIntake = 150,
        liftBasePos = 0;
    private final double liftThreshold = 8;
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

        pivotS = hardware.sch0;
        clawS = hardware.sch1;
        leftArmS = hardware.sch2;
        rightArmS = hardware.sch3;

        liftMotor = hardware.mch0;
        outtakeHardwareSetUp(); // this can now be called from here because the objects initialize at hardware
    }

    public void outtakeHardwareSetUp()
    {
        // if we need to reverse anything
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

    public void armState(OuttakeArmServoState state)
    {
        switch (state)
        {
            case READY:
                leftArmS.setPosition(leftArmReadyPos);
                rightArmS.setPosition(rightArmReadyPos);
                break;
            case TRANSFER:
                leftArmS.setPosition(leftArmTransferPos);
                rightArmS.setPosition(rightArmTransferPos);
                break;
            case SAMPLE:
                leftArmS.setPosition(leftArmSamplePos);
                rightArmS.setPosition(rightArmSamplePos);
                break;
            case SPECIMEN:
                leftArmS.setPosition(leftArmSpecimenPos);
                rightArmS.setPosition(rightArmSpecimenPos);
                break;
            case SPECIMEN_SCORE:
                leftArmS.setPosition(leftArmSpecimenScorePos);
                rightArmS.setPosition(rightArmSpecimenScorePos);
                break;
            case INTAKE:
                leftArmS.setPosition(leftArmIntakePos);
                rightArmS.setPosition(rightArmIntakePos);
                break;
        }
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

    public void liftTo(int rotations)
    {
        liftTarget = rotations;
        liftMotor.setPower(intakeSlidesPID.update(rotations, liftPosition, 1));
    }
    public void liftToInternalPID(int rotations)
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


}
