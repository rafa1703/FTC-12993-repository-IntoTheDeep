package org.firstinspires.ftc.teamcode.opmode.auto;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.gvf.MecanumDrive;
import org.firstinspires.ftc.teamcode.gvf.trajectories.Trajectory;
import org.firstinspires.ftc.teamcode.gvf.trajectories.TrajectoryBuilder;
import org.firstinspires.ftc.teamcode.gvf.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.firstinspires.ftc.teamcode.opmode.teleop.PrometheusDrive;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

@Autonomous(name = "1+3 Close", group = "Close")
public class SampleAuto extends LinearOpMode
{

    enum autoState {
        PRELOAD_DEPOSIT,
        INTAKE,
        INTAKE_SUB,
        INTAKE_DROP,
        TRANSFER_START,
        TRANSFER_END,
        DEPOSIT,
        DROP,
        PARK,
        IDLE
    }
    ElapsedTime GlobalTimer;
    autoState state = autoState.PRELOAD_DEPOSIT;
    GeneralHardware hardware;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    PathsClose trajectories = new PathsClose();
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    double globalTimer, sequenceTimer, intakeClipTimer;
    int cycle = 0;
    boolean intaked = false;
    double yPosition, xPosition;
    boolean dropped = false;
    double intakeSubTarget = 8;
    Trajectory depositTrajectory;
    Trajectory reverseTrajectory = null;

    @Override
    public void runOpMode() throws InterruptedException
    {
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.Red, true);
        hardware.drive.setRunMode(MecanumDrive.RunMode.Vector);
        hardware.drive.getLocalizer().setPose(trajectories.closeStartPose);
        intakeSubsystem = new IntakeSubsystem(hardware);
        outtakeSubsystem = new OuttakeSubsystem(hardware);
        GlobalTimer = new ElapsedTime(System.nanoTime());
        globalTimer = GlobalTimer.milliseconds();
        resetTimer();

        while (!isStarted())
        {
            intakeSubsystem.intakeFilter = IntakeSubsystem.IntakeFilter.NEUTRAL;
            if (delay(1500)) outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
            else outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);

            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.READY);
            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN);
            outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.SPECIMEN_HIGH);

            intakeSubsystem.intakeChute(IntakeSubsystem.IntakeChuteServoState.UP);
            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.LOW);
            intakeSubsystem.intakeFlap(IntakeSubsystem.IntakeFlapServoState.READY);
            intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.HOLD);
            globalTimer = GlobalTimer.milliseconds();
        }
        waitForStart();
        globalTimer = GlobalTimer.milliseconds();
        resetTimer();
        hardware.resetCacheHubs();
        while (opModeIsActive())
        {
            globalTimer = GlobalTimer.milliseconds();
            intakeSubsystem.intakeReads(state == autoState.INTAKE || state == autoState.INTAKE_SUB || state == autoState.TRANSFER_START); // we dont need the color sensor in this auto
            outtakeSubsystem.outtakeReads();

            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            autoSequence();
            hardware.update();
            Pose poseEstimate = hardware.drive.getPoseEstimate();
            yPosition = poseEstimate.getY();
            xPosition = poseEstimate.getX();
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
                if (dropped && delay(200))
                {
                    state = autoState.INTAKE;
                    cycle++;
                    resetTimer();
                    break;
                }
                hardware.drive.followTrajectorySplineHeading(trajectories.preloadTrajectory);
                if (!dropped)
                {
                    if (delay(40))
                    {
                        outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.SPECIMEN_HIGH);
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN);
                        outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SPECIMEN);

                        if (delay(400))
                        {
                            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HIGH);
                        }
                        if (trajectories.preloadTrajectory.isFinished() && hardware.drive.stopped() && delay(500))
                        {
                            outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                            dropped = true;
                            resetTimer();
                        }
                    }
                }
                else // we want to wait for the arm to lower after we release to start driving
                {
                    if (delay(100))
                    {
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.READY);
                    }
                }
                break;
            case INTAKE:
                if (intakeSubsystem.getColorValue() > 500 && delay(190))
                {
                    state =  autoState.TRANSFER_START;
                    intaked = false;
                    resetTimer();
                    break;
                }
                switch (cycle)
                {
                    case 1:
                        hardware.drive.followTrajectorySplineHeading(trajectories.submersibleToFirstIntake);
                        break;
                    case 2:
                        hardware.drive.followTrajectorySplineHeading(trajectories.secondIntake);
                        break;
                    case 3:
                        hardware.drive.followTrajectorySplineHeading(trajectories.thirdIntake);
                        break;
                }
                if (!outtakeSubsystem.liftAtBase()) // this just continue making sure the lift is going down after drop
                    outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftBasePos);
                if (delay(40))
                {
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.LOW);
                    if (delay(90))
                        intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                    if (((trajectories.submersibleToFirstIntake.isFinished() && cycle == 1) || (yPosition > -20 && cycle != 1)) && delay(200))
                    {
                        intakeSubsystem.intakeSlideInternalPID(IntakeSubsystem.slideAutoFar);
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                    }
                    else intakeSubsystem.intakeSlideInternalPID(IntakeSubsystem.slideTeleBase);
                }
                break;
            case INTAKE_SUB:
                hardware.drive.followTrajectorySplineHeading(trajectories.submersibleIntake);
                if (delay(140))
                {
                    if (xPosition > -25) // this should be the actual trigger condition
                    {
                        intakeSubsystem.intakeSlideInternalPID(intakeSubTarget);
                        if (intakeSubsystem.slideReached(intakeSubTarget))
                        {
                            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.LOW);
                        }
                        if (delay(200))
                        {
                            if (intakeSubsystem.getColorValue() > 500)
                            {
                                if (intakeSubsystem.colorLogic())
                                {
                                    state = autoState.TRANSFER_START;
                                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HIGH);
                                    resetTimer();
                                    break;
                                } else
                                {
                                    state = autoState.INTAKE_DROP;
                                    intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.OFF);
                                    resetTimer();
                                    break;
                                }
                            }

                        } else intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                    }
                }
                break;
            case INTAKE_DROP:
                if (intakeSubsystem.getColorValue() < 600 && delay(2000))
                {
                    intakeSubsystem.intakeChute(IntakeSubsystem.IntakeChuteServoState.UP);
                    intakeSubTarget += 4;
                    state = autoState.INTAKE_SUB;
                    resetTimer();
                    break;
                }
                if (delay(400))
                    intakeSubsystem.intakeSlideInternalPID(intakeSubTarget + 5);
                intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.LOW);

                if (delay(50))
                {
                    intakeSubsystem.intakeChute(IntakeSubsystem.IntakeChuteServoState.DROP);
                    if (delay(500))
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                    else if (delay(350)) // this assumes that the sample is stuck in the chute so we spin the intake
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                    else
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.DROP); // this stop the motor until it drop
                }
                break;
            case TRANSFER_START:
                if (delay(1000))
                {
                    state = autoState.TRANSFER_END;
                    resetTimer();
                    break;
                }
                if (delay(40))
                {
                    intakeClipHoldLogicWithoutPowerCutout(IntakeSubsystem.slideTransfer, 5);
                    outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftBasePos);
                    outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.READY);
                    if (delay(120))
                    {
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER);
                        outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.TRANSFER);
                        outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                    }
                    if (intakeSubsystem.isSlidesAtBase() && delay(140))
                    {
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HIGH);
                    }
                }
                break;
            case TRANSFER_END:
                if (delay(430))
                {
                    state = autoState.DEPOSIT;
                    resetTimer();
                    break;
                }
                if (delay(20))
                {
                    if (delay(140))
                    {
                        outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                    }
                    if (delay(200))
                    {
                        outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.TRANSFER_FINISH);
                    }
                    if (delay(230))
                    {
                        intakeClipHoldLogic(IntakeSubsystem.slideTransfer, 5); // here we can cut down the power as transfer has already happened
                    }
                    else outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.TRANSFER);
                    if (delay(290))
                    {
                        outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.TRANSFER_FINISH);
                    }
                    if (delay(350))
                    {
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.LOW);
                    }
                }
                break;
            case DEPOSIT:
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
                depositTrajectory = null;
                switch (cycle)
                {
                    case 1:
                        depositTrajectory = trajectories.firstDeposit;
//                        hardware.drive.followTrajectorySplineHeading(trajectories.firstDeposit);
                        break;
                    case 2:
//                        hardware.drive.followTrajectorySplineHeading(trajectories.secondDeposit);
                        depositTrajectory = trajectories.secondDeposit;
                        break;
                    case 3:
                        depositTrajectory = trajectories.thirdDeposit;
//                        hardware.drive.followTrajectorySplineHeading(trajectories.thirdDeposit);
                        break;
                    case 4:
                        depositTrajectory = trajectories.forthDeposit;
//                        hardware.drive.followTrajectorySplineHeading(trajectories.forthDeposit);
                        break;
                }
                if (depositTrajectory != null)
                    hardware.drive.followTrajectorySplineHeading(depositTrajectory);
                if (delay(150))
                {
                    if (yPosition > -30)
                    {
                        outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftHighBucketPos);
                    }
                    if (delay(290))
                    {
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SAMPLE);
                        outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SAMPLE);
                    }
                    if (delay(400)) outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.SAMPLE);
                    else outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.OVER_THE_TOP);
                }
                break;
            case DROP:
                if (delay(250))
                {
                    state = cycle == 3 ? autoState.PARK : autoState.INTAKE; // add the sub cycle here
                    cycle++;
                    reverseTrajectory = null;
                    resetTimer();
                    break;
                }
                // if this get caught might have to dynamically generate a trajectory we go back
                if (reverseTrajectory == null) // this should make it so it only runs once
                {
                    Pose reversedPose = depositTrajectory.getFinalPose();
                    reversedPose = new Pose(reversedPose.getX() + 3, reversedPose.getY() + 3, reversedPose.getHeading());
                    reverseTrajectory = new TrajectoryBuilder()
                            .addBezierSegment(
                                    depositTrajectory.getFinalPose().toPoint(),
                                    reversedPose.toPoint()
                            )
                            .addFinalPose(reversedPose)
                            .build();
                }
                //hardware.drive.followTrajectorySplineHeading(reverseTrajectory);

                outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                if (delay(120))
                {
                    // we might need to turn the wrist before the arm goes back as the clearance is
                    //outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.READY);
                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.READY);
                }
                if (delay(200))
                {
                    outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftBasePos);
                    outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.READY);
                    outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.READY);
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
    public void intakeClipHoldLogic(double slideToPosition, int closeThreshold)
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
    public void intakeClipHoldLogicWithoutPowerCutout(double slideToPosition, int closeThreshold){
        if (intakeSubsystem.slidePosition < closeThreshold) {
            if (globalTimer - intakeClipTimer > 100){
                intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.HOLD);
                intakeSubsystem.intakeSlideInternalPID(0);
            } else {
                intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.HOLD);
                intakeSubsystem.intakeSlideInternalPID(slideToPosition);
            }
        } else {
            intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
            intakeSubsystem.intakeSlideInternalPID(slideToPosition);
            //
            intakeClipTimer = globalTimer;
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
