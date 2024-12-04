package org.firstinspires.ftc.teamcode.opmode.auto;


import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTransfer;

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
        AFTER_EXTENDO,
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
    double yPosition, xPosition, headingPosition;
    double headingErrorToEndPose;
    boolean dropped = false;
    double intakeSubTarget = 8;
    Trajectory depositTrajectory;
    Trajectory reverseTrajectory = null;

    @Override
    public void runOpMode() throws InterruptedException
    {
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.Red, true);
        hardware.drive.setRunMode(MecanumDrive.RunMode.Vector);
        hardware.drive.getLocalizer().setOffSet(trajectories.closeStartPose);
        intakeSubsystem = new IntakeSubsystem(hardware);
        outtakeSubsystem = new OuttakeSubsystem(hardware);
        GlobalTimer = new ElapsedTime(System.nanoTime());
        globalTimer = GlobalTimer.milliseconds();
        resetTimer();

        while (!isStarted())
        {
            intakeSubsystem.intakeFilter = IntakeSubsystem.IntakeFilter.NEUTRAL;
            outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
            if (delay(400))
            {
                outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.TRANSFER_FINISH);
                outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER_FINISH);
                outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.HIGH);
            }

            intakeSubsystem.intakeChute(IntakeSubsystem.IntakeChuteServoState.UP);
            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HIGH);
            intakeSubsystem.intakeFlap(IntakeSubsystem.IntakeFlapServoState.DOWN);
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
            headingPosition = poseEstimate.getHeading();
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
                    if (delay(0))
                        outtakeSubsystem.liftToInternalPIDTicks(1655);
                    if (delay(40))
                    {
                        outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.SAMPLE);
                        if(delay(100))
                        {
                            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SAMPLE);
                            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SAMPLE);
                        }
                        if (trajectories.preloadTrajectory.isFinished() && delay(400))
                        {
                            outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.INTAKE);
                            dropped = true;
                            resetTimer();
                        }
                    }
                }
                break;
            case INTAKE:
                if (intakeSubsystem.getColorValue() > 500 && delay(190))
                {
                    state =  autoState.AFTER_EXTENDO;
                    intaked = false;
                    resetTimer();
                    break;
                }
                Trajectory intakeTrajectory = null;
                switch (cycle)
                {
                    case 1:
                        intakeTrajectory = trajectories.firstIntake;
                        break;
                    case 2:
                        intakeTrajectory = trajectories.secondIntake;
                        break;
                    case 3:
                        intakeTrajectory = trajectories.thirdIntake;
                        break;
                }


                if (intakeTrajectory != null)
                {
                    hardware.drive.followTrajectorySplineHeading(intakeTrajectory);
                    headingErrorToEndPose = Math.toDegrees(Math.abs(headingPosition - intakeTrajectory.getFinalPose().getHeading()));
                    boolean reachedFinalHeading =  headingErrorToEndPose < 2;
                    if (delay(0))
                    {
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER);
                        outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                        outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.TRANSFER);
                        outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.MIDDLE);
                    }
                    if (delay(40))
                    {
                        if (delay(90))
                        {
                            if (!outtakeSubsystem.liftAtBase()) // this just continue making sure the lift is going down after drop
                                outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftBasePos);
                            intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                        }
                        if (intakeSubsystem.slideOverPosition(10) ||
                                (cycle == 2))
                        {
                            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HALF_DOWN);
                        }
//                        if(delay(1400))
//                            intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
//                        else if (delay(800))
//                            intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);

                        if (reachedFinalHeading)
                        {
                            if (cycle > 1)
                                intakeSubsystem.intakeSlideInternalPID(16);
                            else
                                intakeSubsystem.intakeSlideInternalPID(IntakeSubsystem.slideAutoFar);
                        } else
                            intakeSubsystem.intakeSlideInternalPID(IntakeSubsystem.slideTeleBase);
                    }
                }
                break;
            case AFTER_EXTENDO:
                if (delay(400) && intakeSubsystem.isSlidesAtBase())
                {
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                    state = autoState.TRANSFER_START;
                    resetTimer();
                    break;
                }

                if (intakeSubsystem.slidePosition < 20)
                {
                    intakeSubsystem.intakeSpin(0.7);
                }
                else if (delay(90))
                {
                    intakeSubsystem.intakeSpin(-0.2);
                }
                else intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);

                if (delay(100)) // gives time for the intake to go up
                {
                    outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.TRANSFER);
                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER);
                    outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.MIDDLE);
                    intakeClipHoldLogic(-10, 5);
                }

                break;
            case TRANSFER_START:
                if (delay(850) && outtakeSubsystem.liftAtBase() && intakeSubsystem.isSlidesAtBase())
                {
                    state =autoState.TRANSFER_END;
                    resetTimer();
                    break;
                }
                intakeClipHoldLogicWithoutPowerCutout(slideTransfer, 10); // this controls the intake slides and the clip
                outtakeSubsystem.liftToInternalPIDTicks(-20);
                if (delay(60))
                {
                    outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.TRANSFER);

                    if (delay(230))
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HIGH);
                    if (delay(240))
                    {
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER);
                        outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);

                    }
                    if (delay(400))
                    {
                        outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.TRANSFER);
                    }
                }
                break;
            case TRANSFER_END:
                // so this is when the claw will grip and we are assuming that the slides are at transfer position
                if (delay(490))
                {
                    intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.OFF);
                    intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.HOLD);
                    intakeSubsystem.intakeFlap(IntakeSubsystem.IntakeFlapServoState.DOWN);
                    state = autoState.DEPOSIT;
                    resetTimer();
                    break;
                }
                if ((delay(40)))
                {
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                    if (delay(200))
                    {
                        intakeSubsystem.intakeFlap(IntakeSubsystem.IntakeFlapServoState.TRANSFER);
                    }
                    if (delay(350))
                    {
                        outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.TRANSFER_FINISH); // we might have to wait to go up
                        outtakeSubsystem.liftMotorRawControl(0);
                    }
                    if (delay(390))
                        outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.TRANSFER_FINISH);
                }
                break;
            case DEPOSIT:
                if ((
                        (cycle == 1 && trajectories.firstDeposit.isFinished()) ||
                        (cycle == 2 && trajectories.secondDeposit.isFinished()) ||
                        (cycle == 3 && trajectories.thirdDeposit.isFinished() &&
                                Math.toDegrees(Math.abs(headingPosition - trajectories.thirdDeposit.getFinalPose().getHeading())) < 2) ||
                        (cycle == 4 && trajectories.forthDeposit.isFinished()))
                        && delay(600) && hardware.drive.stopped())
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
                    if (delay(1000))
                        hardware.drive.followTrajectorySplineHeading(depositTrajectory);
                if (delay(150))
                {
                    outtakeSubsystem.liftToInternalPIDTicks(1655);
                    outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.SAMPLE);
                    if (delay(150))
                    {
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SAMPLE);
                        outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SAMPLE);
                    }
                }
                break;
            case DROP:
                if ( cycle == 0 ? delay(250)  : delay(400))
                {
                    state = cycle == 3 ? autoState.PARK : autoState.INTAKE; // add the sub cycle here
                    cycle++;
                    reverseTrajectory = null;
                    resetTimer();
                    break;
                }
//                // if this get caught might have to dynamically generate a trajectory we go back
//                if (reverseTrajectory == null) // this should make it so it only runs once
//                {
//                    Pose reversedPose = depositTrajectory.getFinalPose();
//                    reversedPose = new Pose(reversedPose.getX() + 3, reversedPose.getY() + 3, reversedPose.getHeading());
//                    reverseTrajectory = new TrajectoryBuilder()
//                            .addBezierSegment(
//                                    depositTrajectory.getFinalPose().toPoint(),
//                                    reversedPose.toPoint()
//                            )
//                            .addFinalPose(reversedPose)
//                            .build();
//                }
                //hardware.drive.followTrajectorySplineHeading(reverseTrajectory);

                outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.INTAKE);
//                if (delay(180))
//                {
//                    // we might need to turn the wrist before the arm goes back as the clearance is
//                    //outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.READY);
//                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.READY);
//                }
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
                if (delay(200))
                {
                    outtakeSubsystem.armSetPos(0.43);
                    outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.HIGH);
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                    outtakeSubsystem.liftToInternalPIDTicks(100);
                }
                break;
            case IDLE: // we idle here duuhhh
                outtakeSubsystem.liftToInternalPIDTicks(0);
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
