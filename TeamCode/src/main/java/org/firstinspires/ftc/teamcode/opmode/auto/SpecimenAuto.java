package org.firstinspires.ftc.teamcode.opmode.auto;


import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTransfer;
import static org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware.S;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.gvf.MecanumDrive;
import org.firstinspires.ftc.teamcode.gvf.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

@Autonomous(name = "GVF CLOSE 1 +2 NO PARK", group = "Close")
public class SpecimenAuto extends LinearOpMode
{

    enum autoState {
        PRELOAD_DEPOSIT,
        PUSH,
        INTAKE,
        DEPOSIT_DRIVE,
        DROP,
        PARK,
        IDLE
    }
    ElapsedTime GlobalTimer;
    autoState state = autoState.PRELOAD_DEPOSIT;
    GeneralHardware hardware;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    PathsFar trajectories = new PathsFar();
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    double globalTimer, sequenceTimer, intakeClipTimer;
    int cycle = 0;
    int pushCycle = 0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.Red, true);
        hardware.drive.setRunMode(MecanumDrive.RunMode.Vector);
        hardware.drive.getLocalizer().setPose(trajectories.farStartPose);
        intakeSubsystem = new IntakeSubsystem(hardware);
        outtakeSubsystem = new OuttakeSubsystem(hardware);
        GlobalTimer = new ElapsedTime(System.nanoTime());
        globalTimer = GlobalTimer.milliseconds();
        resetTimer();

        while (!isStarted())
        {
            intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.HOLD);
            if (delay(1000)) outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
            else outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);

            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.READY);
            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN);
            outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.SPECIMEN_HIGH);
            //TODO: think on how to load the preload
            globalTimer = GlobalTimer.milliseconds();
        }
        waitForStart();
        globalTimer = GlobalTimer.milliseconds();
        resetTimer();
        hardware.resetCacheHubs();
        while (opModeIsActive())
        {
            globalTimer = GlobalTimer.milliseconds();
            intakeSubsystem.intakeReads(false); // we dont need the color sensor in this auto
            outtakeSubsystem.outtakeReads();

            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            autoSequence();
            hardware.update();
            Pose poseEstimate = hardware.drive.getPoseEstimate();
            DashboardUtil.drawRobot(fieldOverlay, poseEstimate.toPose2d(), true, "red");
            DashboardUtil.drawRobot(fieldOverlay, hardware.drive.getPredictedPoseEstimate().toPose2d(), true);
            DashboardUtil.drawCurve(fieldOverlay, hardware.drive.trajectoryFollowing);
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("State", state);
            telemetry.addData("Pose", hardware.drive.getPoseEstimate());
            telemetry.update();
        }
    }
    public void autoSequence()
    {
        switch (state)
        {
            case PRELOAD_DEPOSIT:
                if (trajectories.preloadTrajectory.isFinished() && hardware.drive.stopped())
                {
                    state = autoState.DROP;
                    resetTimer();
                    break;
                }
                hardware.drive.followTrajectorySplineHeading(trajectories.preloadTrajectory);
                if (delay(40))
                {
                    outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.SPECIMEN_HIGH);
                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN);
                    outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SPECIMEN);

                    if (delay(400))
                    {
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HIGH);
                    }
                }
                break;
            case PUSH:
                if (trajectories.thirdSampleToHPAndIntake.isFinished() && hardware.drive.stopped() && delay(0))
                {
                    state = autoState.DEPOSIT_DRIVE;
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                    resetTimer();
                    break;
                }
                switch (pushCycle)
                {
                    case 0:
                        if (trajectories.submersibleToSamplesTrajectory.isFinished()) pushCycle++;
                        hardware.drive.followTrajectorySplineHeading(trajectories.submersibleToSamplesTrajectory);
                        break;
                    case 1:
                        if (trajectories.firstSampleToHP.isFinished()) pushCycle++;
                        hardware.drive.followTrajectorySplineHeading(trajectories.firstSampleToHP);
                        break;
                    case 2:
                        if (trajectories.hpToSecondSample.isFinished()) pushCycle++;
                        hardware.drive.followTrajectorySplineHeading(trajectories.hpToSecondSample);
                        break;
                    case 3:
                        if (trajectories.secondSampleToHP.isFinished()) pushCycle++;
                        hardware.drive.followTrajectorySplineHeading(trajectories.secondSampleToHP);
                        break;
                    case 4:
                        if (trajectories.hpToThirdSample.isFinished())
                        {
                            pushCycle++;
                            resetTimer();
                        }
                        hardware.drive.followTrajectorySplineHeading(trajectories.hpToThirdSample);
                        break;
                    case 5:
                        hardware.drive.followTrajectorySplineHeading(trajectories.thirdSampleToHPAndIntake);

                        if (delay(130))
                        {
                            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.INTAKE);
                            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.PERPENDICULAR);
                        }
                        if (delay(230))
                            outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.INTAKE);
                        else outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.OVER_THE_TOP);
                        break;
                }
                break;
            case INTAKE:
                if (intakeSubsystem.getColorValue() > 800)
                {
                    state =  autoState.DEPOSIT_DRIVE;
                    resetTimer();
                    break;
                }
                switch (cycle)
                {
                    case 1:
                        hardware.drive.followTrajectorySplineHeading(trajectories.firstIntake);
                        break;
                    case 2:
                        hardware.drive.followTrajectorySplineHeading(trajectories.secondIntake);
                        break;
                    case 3:
                        hardware.drive.followTrajectorySplineHeading(trajectories.thirdIntake);
                        break;
                }
                if (delay(40))
                {
                    if (delay(140))
                    {
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.INTAKE);
                        outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                    }
                    if (delay(240)) outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.INTAKE);
                    else outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.OVER_THE_TOP);
                    if (delay(300)) outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.INTAKE);
                    else outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.PERPENDICULAR);

                    if (delay(550))
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.LOW);
                }
                break;
            case DEPOSIT_DRIVE:
                if ((
                        (cycle == 1 && trajectories.firstDeposit.isFinished()) ||
                        (cycle == 2 && trajectories.secondDeposit.isFinished()) ||
                        (cycle == 3 && trajectories.thirdDeposit.isFinished()) ||
                        (cycle == 4 && trajectories.forthDeposit.isFinished()))
                        && delay(400) && hardware.drive.stopped())
                {
                    state = autoState.DROP;
                    resetTimer();
                    break;
                }
                switch (cycle)
                {
                    case 1:
                        hardware.drive.followTrajectorySplineHeading(trajectories.firstDeposit);
                        break;
                    case 2:
                        hardware.drive.followTrajectorySplineHeading(trajectories.secondDeposit);
                        break;
                    case 3:
                        hardware.drive.followTrajectorySplineHeading(trajectories.thirdDeposit);
                        break;
                    case 4:
                        hardware.drive.followTrajectorySplineHeading(trajectories.forthDeposit);
                        break;
                }
                if (delay(100))
                {
                    outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SPECIMEN);
                    if (delay(200))
                    {
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN);
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HIGH);
                    }
                    if (delay(300))
                    {
                        outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.SPECIMEN_HIGH);
                    }
                    else outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.OVER_THE_TOP);
                }
                break;
            case DROP:
                if (delay(230))
                {
                    state = cycle == 0 ? autoState.PUSH : cycle == 4 ? autoState.PARK : autoState.INTAKE;
                    cycle++;
                    resetTimer();
                    break;
                }
                outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                if (delay(120))
                {
                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.READY);
                }
                break;
            case PARK:
                if (trajectories.parkTrajectory.isFinished() && delay(1000))
                {
                    state = autoState.IDLE;
                    resetTimer();
                    break;
                }
                hardware.drive.followTrajectorySplineHeading(trajectories.parkTrajectory);
                break;
            case IDLE: // we idle here duuhhh
                break;
        }
    }
    public void intakeClipHoldLogic(int slideToPosition, int closeThreshold)
    {
        if (intakeSubsystem.slidePosition < closeThreshold)
        {
            if (globalTimer - intakeClipTimer > 90)
            {
                intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.HOLD);
                intakeSubsystem.intakeSlideMotorRawControl(0);
            } else
            {
                intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.HOLD);
                intakeSubsystem.intakeSlideInternalPID(slideToPosition);
            }
        } else
        {
            intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
            intakeSubsystem.intakeSlideInternalPID(slideToPosition);
            intakeClipTimer = globalTimer;
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
