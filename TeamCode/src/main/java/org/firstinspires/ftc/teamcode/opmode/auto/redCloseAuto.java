package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

public class redCloseAuto extends LinearOpMode
{
    // IDK what this class does yet, as I don't want to code this tbh :0
    enum autoState {
        DRIVE_DEPOSIT,
        DEPOSIT,
        DRIVE_SAMPLE,
        DRIVE_HP,
        DRIVE_PARK,
        IDLE
    }
    ElapsedTime GlobalTimer;
    autoState state = autoState.DEPOSIT;
    GeneralHardware hardware;
    Paths trajectories = new Paths();
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    double globalTimer, sequenceTimer, intakeClipTimer;

    @Override
    public void runOpMode() throws InterruptedException
    {
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.Red, true);
        hardware.startThreads(this);
        intakeSubsystem = new IntakeSubsystem(hardware);
        outtakeSubsystem = new OuttakeSubsystem(hardware);
        GlobalTimer = new ElapsedTime(System.nanoTime());
        globalTimer = GlobalTimer.milliseconds();
        resetTimer();
        while (!isStarted())
        {
            intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.HOLD);
            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HIGH);
            outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
            if (delay(1000)) outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.INTAKE);
        }
        waitForStart();
        while (opModeIsActive())
        {
            globalTimer = GlobalTimer.milliseconds();
            autoSequence();
            hardware.update();
            telemetry.update();
        }
    }
    public void autoSequence()
    {
        switch (state)
        {
            case DRIVE_DEPOSIT:
                if(trajectories.depositFarTrajectory.isFinished())
                {
                    state = autoState.DEPOSIT;
                    resetTimer();
                }
                hardware.drive.followTrajectory(trajectories.depositFarTrajectory);
                break;
            case DEPOSIT:
                if (delay(1000))
                {
                    state = autoState.DRIVE_HP;
                    resetTimer();
                }
                if (delay(250)) outtakeLiftPresets(false, false, -40); // this actually runs the lift
                else outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftHighBarPos);
                if (delay(150)) outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN_SCORE);
                if (delay( 500)) outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                if (delay( 600))
                {
                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.READY);
                    outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftBasePos);
                }
                break;
            case DRIVE_SAMPLE:
                if (trajectories.depositFarTrajectory.isFinished())
                {
                    state = autoState.DRIVE_HP;
                    resetTimer();
                }
                if(delay(40)) hardware.drive.followTrajectorySplineHeading(trajectories.depositFarTrajectory);
                break;
            case DRIVE_HP:
                if (trajectories.samplesToHPFarTrajectory.isFinished())
                {
                    state = autoState.DRIVE_PARK;
                    resetTimer();
                }
                if (delay(40)) hardware.drive.followTrajectorySplineHeading(trajectories.samplesToHPFarTrajectory);
                break;
            case DRIVE_PARK:
                if(trajectories.hpToParkFarTrajectory.isFinished())
                {
                    state = autoState.IDLE;
                    resetTimer();
                }
                if (delay(40)) hardware.drive.followTrajectorySplineHeading(trajectories.hpToParkFarTrajectory);
                break;
        }
    }
    public void outtakeLiftPresets(boolean isSample, boolean isLow, int offSet)
    {
        if (isSample)
        {
            if (isLow) outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftLowBucketPos + offSet);
            else outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftHighBucketPos + offSet);
        }
        else
        {
            if (isLow) outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftLowBarPos + offSet);
            else outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftHighBarPos + offSet);
        }
    }
    public boolean delay(double delayTime)
    {
        return (globalTimer - sequenceTimer) > delayTime;
    }

    public void resetTimer()
    {
        sequenceTimer = globalTimer;
    }
}
