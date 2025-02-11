package org.firstinspires.ftc.teamcode.opmode.auto;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.gvf.MecanumDrive;
import org.firstinspires.ftc.teamcode.gvf.trajectories.Trajectory;
import org.firstinspires.ftc.teamcode.gvf.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.firstinspires.ftc.teamcode.opmode.auto.paths.PathsFarExtra;
import org.firstinspires.ftc.teamcode.system.accessory.ToggleUpOrDownCircular;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

import java.util.Arrays;

@Autonomous(name = "6+0 Far", group = "Far")
public class SpecimenHighCycleAuto extends LinearOpMode
{

    enum autoState {
        PRELOAD_DEPOSIT,
        AFTER_SUB_INTAKE,
        EJECTION_TO_HP,
        SAMPLE_PICKUP,
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
    PathsFarExtra trajectories = new PathsFarExtra();
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    double globalTimer, sequenceTimer, intakeClipTimer;
    int cycle = 0;
    int pickupCycle = 0;
    boolean intakedSpec = false;
    boolean attemptedIntake = false;
    boolean startedExtending = false;
    double xPosition, yPosition, headingPosition;
    double headingErrorToEndPose;
    ToggleUpOrDownCircular rowToggle = new ToggleUpOrDownCircular(1, 1, 0, 2);
    ToggleUpOrDownCircular columnToggle = new ToggleUpOrDownCircular(1, 1, 3, 5);

    @Override
    public void runOpMode() throws InterruptedException
    {
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.Red, true);
        hardware.drive.setRunMode(MecanumDrive.RunMode.Vector);
        hardware.drive.getLocalizer().setOffSet(trajectories.farStartPose);
        intakeSubsystem = new IntakeSubsystem(hardware);
        outtakeSubsystem = new OuttakeSubsystem(hardware);
        GlobalTimer = new ElapsedTime(System.nanoTime());
        globalTimer = GlobalTimer.milliseconds();

        intakeSubsystem.intakeFilter = IntakeSubsystem.IntakeFilter.SIDE_ONLY;
        resetTimer();

        while (!isStarted())
        {
            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HIGH);
            intakeSubsystem.intakeChute(IntakeSubsystem.IntakeChuteServoState.UP);
            intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.HOLD);
            outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.READY);
            if (delay(1000))
            {
                intakeSubsystem.intakeFlap(IntakeSubsystem.IntakeFlapServoState.DOWN);
                outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.SPECIMEN_HIGH);
            }
            if (delay(1500))
            {
                outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN);
            }

            rowToggle.upToggle(gamepad1.dpad_up);
            rowToggle.downToggle(gamepad1.dpad_down, 1);
            columnToggle.upToggle(gamepad1.dpad_right);
            columnToggle.downToggle(gamepad1.dpad_left, 1);
            drawSub(telemetry, rowToggle.OffsetTargetPosition, columnToggle.OffsetTargetPosition);
            telemetry.update();
            globalTimer = GlobalTimer.milliseconds();
        }
        waitForStart();
        globalTimer = GlobalTimer.milliseconds();
        resetTimer();
        hardware.resetCacheHubs();
        while (opModeIsActive())
        {
            globalTimer = GlobalTimer.milliseconds();
            intakeSubsystem.intakeReads(state == autoState.PRELOAD_DEPOSIT || state == autoState.EJECTION_TO_HP || state == autoState.SAMPLE_PICKUP); // we dont need the color sensor in this auto
            outtakeSubsystem.outtakeReads();

            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            autoSequence();
            hardware.update();
            Pose poseEstimate = hardware.drive.getPoseEstimate();
            xPosition = poseEstimate.getX();
            yPosition = poseEstimate.getY();
            headingPosition = poseEstimate.getHeading();

            DashboardUtil.drawRobot(fieldOverlay, poseEstimate.toPose2d(), true, "red");
            DashboardUtil.drawRobot(fieldOverlay, hardware.drive.getPredictedPoseEstimate().toPose2d(), true, "black");
            DashboardUtil.drawCurve(fieldOverlay, hardware.drive.trajectoryFollowing);
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("State", state);
            telemetry.addData("Cycle", cycle);
            telemetry.addData("Reached third intake", trajectories.thirdIntake.isFinished());
            telemetry.addData("Pickup cycle", pickupCycle);
            telemetry.addData("Is slide over 14in", intakeSubsystem.slideOverPosition(14));
            telemetry.addData("HeadingPosition", headingPosition);
            telemetry.addData("Heading Error To End Of Trajectory", headingErrorToEndPose);
            telemetry.addData("Pose", hardware.drive.getPoseEstimate());
            telemetry.update();
        }
    }
    public void autoSequence()
    {
        switch (state)
        {
            case PRELOAD_DEPOSIT:
                if (trajectories.preloadTrajectory.isFinished() && attemptedIntake)
                {
                    state = autoState.DROP;
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HIGH);
                    cycle ++;
                    attemptedIntake = false;
                    resetTimer();
                    break;
                }
                hardware.drive.followTrajectorySplineHeading(trajectories.preloadTrajectory);
                if (delay(0))
                {
                    intakeSubsystem.intakeFlap(IntakeSubsystem.IntakeFlapServoState.DOWN);
                    intakeSubsystem.intakeChute(IntakeSubsystem.IntakeChuteServoState.UP);
                    outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.SPECIMEN_HIGH);
                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN);
                    outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SPECIMEN);
                    outtakeSubsystem.liftToInternalPIDTicks(670);
                    //outtakeLiftPresets(false, false);
                }
                if (delay(50))
                {
                    intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                }
                if (delay(140))
                {
                    if (yPosition > -28) // this should be the actual trigger condition
                    {
                        intakeSubsystem.intakeSlideInternalPID(8);
                        if (intakeSubsystem.slideReached(8))
                        {
                            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.LOW);
                        }
                        if (intakeSubsystem.getColorValue() > 500)
                        {
                            if (intakeSubsystem.colorLogic())
                            {
                                intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HIGH); // this shouldn't be a problem as the state changes after
                            } else
                                intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                            attemptedIntake = true;

                        } else intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                    }
                }
                break;
            case EJECTION_TO_HP:
                Trajectory ejectionTrajectory = null;
                switch (pickupCycle)
                {
                    case 0:
                        ejectionTrajectory = trajectories.subToFirstE;
                        break;
                    case 1:
                        ejectionTrajectory = trajectories.secondE;
                        break;
                    case 2:
                        ejectionTrajectory = trajectories.thirdE;
                        break;
                    case 3:
                        ejectionTrajectory = trajectories.forthE; // this path is horrible
                        break;
                }
                if (ejectionTrajectory != null)
                {
                    hardware.drive.followTrajectorySplineHeading(ejectionTrajectory);
                    if (delay(40))
                    {
                        headingErrorToEndPose = Math.toDegrees(Math.abs(headingPosition - ejectionTrajectory.getFinalPose().getHeading()));
                        boolean reachedFinalHeading =  headingErrorToEndPose < 2;

                        if (pickupCycle == 0 && !(headingErrorToEndPose < 15) && delay(300)) // this makes sure we reset after the preload
                        {
                            outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.INTAKE);
                            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.INTAKE);
                            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.READY);
                            outtakeSubsystem.liftToInternalPIDTicks(0);
                        }
                        intakeSubsystem.intakeChute(IntakeSubsystem.IntakeChuteServoState.UP);
                        if (
                                reachedFinalHeading &&
                                ((pickupCycle == 0 && intakeSubsystem.slideOverPosition(15)) ||
                                (pickupCycle == 1 && intakeSubsystem.slideOverPosition(15)) ||
                                (pickupCycle == 2 && intakeSubsystem.slideOverPosition(17.5)) ||
                                (pickupCycle == 3 && intakeSubsystem.slideOverPosition(15))) // should probably add reachedFinalHeadingHere
                        )
                        {
                            intakeSubsystem.intakeFlap(IntakeSubsystem.IntakeFlapServoState.TRANSFER);
                            intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                        }
//                        else if (ejectionTrajectory.isFinished() && reachedFinalHeading && intakeSubsystem.ticksToInchesSlidesMotor(intakeSubsystem.slidePosition) > 16)
//                        {
//                            intakeSubsystem.intakeFlap(IntakeSubsystem.IntakeFlapServoState.TRANSFER);
//                            intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
//                        }

                        if ((pickupCycle == 0 && headingErrorToEndPose < 10) || headingErrorToEndPose < 15)
                        {
                            switch (pickupCycle) // leave as a switch until i know everything works
                            {
                                case 0:
                                    intakeSubsystem.intakeSlideInternalPID(18.5);
                                    break;
                                case 1:
                                    intakeSubsystem.intakeSlideInternalPID(16);
                                    break;
                                case 2:
                                    intakeSubsystem.intakeSlideInternalPID(18.5);
                                    break;
                                case 3:
                                    intakeSubsystem.intakeSlideInternalPID(18.5);
                                    break;
                            }
                        }
                        else if (pickupCycle == 0) intakeSubsystem.intakeSlideInternalPID(0);
                        else if (pickupCycle != 3) intakeSubsystem.intakeSlideInternalPID(5);
                        //if (delay(600) && ejectionTrajectory.isFinished() && reachedFinalHeading && intakeSubsystem.getColorValue() < 100)
                        if (delay(600) && intakeSubsystem.getColorValue() < 100)
                        {
                            state = pickupCycle == 3 ? autoState.INTAKE : autoState.SAMPLE_PICKUP;
                            intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.OFF);
                            intakeSubsystem.intakeFlap(IntakeSubsystem.IntakeFlapServoState.DOWN);
                            intakeSubsystem.intakeSlideInternalPID(0);
                            resetTimer();
                            break;
                        }
                    }
                }
                break;
            case SAMPLE_PICKUP:
                Trajectory pickupTrajectory = null;
                switch (pickupCycle)
                {
                    case 0:
                        pickupTrajectory = trajectories.firstSP;
                        break;
                    case 1:
                        pickupTrajectory = trajectories.secondSP;
                        break;
                    case 2:
                        pickupTrajectory = trajectories.thirdSP;
                        break;
                }
                if (pickupTrajectory != null)
                {
                    hardware.drive.followTrajectorySplineHeading(pickupTrajectory);
                    if (delay(40))
                    {

                        headingErrorToEndPose = Math.toDegrees(Math.abs(headingPosition - pickupTrajectory.getFinalPose().getHeading()));
                        boolean reachedFinalHeading =  headingErrorToEndPose < 2;

                        if ( headingErrorToEndPose < 7)
                        {
                            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HALF_DOWN);
                        }
                        if (delay(140))
                        {
                            intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                        }


                        if (reachedFinalHeading || startedExtending)//(headingPosition > (pickupCycle == 0 ? Math.toRadians(25) : Math.toRadians(15)))
                        {
                            switch (pickupCycle)
                            {
                                case 0:
                                    intakeSubsystem.intakeSlideInternalPID(13);
                                    break;
                                case 1:
                                    intakeSubsystem.intakeSlideInternalPID(8);
                                    break;
                                case 2:
                                    intakeSubsystem.intakeSlideInternalPID(16.5);
                                    break;
                            }
                            startedExtending = true;
                        }
                        else intakeSubsystem.intakeSlideInternalPID(0);

                        if ((delay(300) && intakeSubsystem.getColorValue() > 500 && pickupTrajectory.isFinished())
                                || delay(3000)
                        )
                        {
                            state = autoState.EJECTION_TO_HP;
                            startedExtending = false;
                            pickupCycle++;
                            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HIGH);
                            resetTimer();
                            break;
                        }
                    }
                }
                break;
            case INTAKE:
                if (intakedSpec && delay(190) && hardware.drive.stopped())
                {
                    state =  autoState.DEPOSIT_DRIVE;
                    intakedSpec = false;
                    resetTimer();
                    break;
                }
                if (!intakedSpec)
                {

                    if (cycle == 2)
                    {
                        intakeSubsystem.intakeFlap(IntakeSubsystem.IntakeFlapServoState.TRANSFER);
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                    }
                    switch (cycle)
                    {
                        case 2: // this case has to be a little different
                            headingErrorToEndPose = Math.toDegrees(Math.abs(headingPosition - trajectories.firstIntakePart1.getFinalPose().getHeading()));
                            if (headingErrorToEndPose < 2 && trajectories.firstIntakePart1.isFinished())
                                hardware.drive.followTrajectorySplineHeading(trajectories.firstIntakePart2);
                            else
                                hardware.drive.followTrajectorySplineHeading(trajectories.firstIntakePart1);
                            if (trajectories.firstIntakePart2.isFinished() && hardware.drive.stopped())
                            {
                                outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                                intakedSpec = true;
                                resetTimer();
                            }
                            break;
                        case 3:
                            hardware.drive.followTrajectorySplineHeading(trajectories.secondIntake);
                            if (trajectories.secondIntake.isFinished() && hardware.drive.stopped())
                            {
                                outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                                intakedSpec = true;
                                resetTimer();
                            }
                            break;
                        case 4:
                            hardware.drive.followTrajectorySplineHeading(trajectories.thirdIntake);
                            if (trajectories.thirdIntake.isFinished() && hardware.drive.stopped())
                            {
                                outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                                intakedSpec = true;
                                resetTimer();
                            }
                            break;
                        case 5:
                            hardware.drive.followTrajectorySplineHeading(trajectories.forthIntake);
                            if (trajectories.forthIntake.isFinished() && hardware.drive.stopped())
                            {
                                outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                                intakedSpec = true;
                                resetTimer();
                            }
                            break;
                        case 6:
                            hardware.drive.followTrajectorySplineHeading(trajectories.fifthIntake);
                            if (trajectories.fifthIntake.isFinished() && hardware.drive.stopped())
                            {
                                outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                                intakedSpec = true;
                                resetTimer();
                            }
                            break;
                    }

                    if (delay(40))
                    {
                        if (delay(140))
                        {
                            outtakeSubsystem.liftToInternalPIDTicks(0);
                            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.INTAKE);
                            outtakeSubsystem.clawSetPos(1);
                        }
                        if (delay(300))
                        {
                            //outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.INTAKE);
                            outtakeSubsystem.railSetPos(0.57);
                            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.INTAKE);
                        }

                        //if (delay(550))
                            //intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.LOW);
                    }
                }
                else
                {
                    if (delay(90))
                    {
                        outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.HIGH);
                        outtakeSubsystem.liftToInternalPIDTicks(500);
                        if (cycle == 2)
                        {
                            intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.OFF);
                        }
                    }
                }
                break;
            case DEPOSIT_DRIVE:
                if (( // THIS ONLY RUNS ON CYCLE 1 ONWARDS
                        (cycle == 2 && trajectories.firstDeposit.isFinished()) ||
                        (cycle == 3 && trajectories.secondDeposit.isFinished()) ||
                        (cycle == 4 && trajectories.thirdDeposit.isFinished()) ||
                        (cycle == 5 && trajectories.forthDeposit.isFinished()) ||
                        (cycle == 6 && trajectories.fifthDeposit.isFinished()))
                        && delay(400))
                {
                    state = autoState.DROP;
                    resetTimer();
                    break;
                }
                switch (cycle)
                {
                    case 2:
                        hardware.drive.followTrajectorySplineHeading(trajectories.firstDeposit);
                        break;
                    case 3:
                        hardware.drive.followTrajectorySplineHeading(trajectories.secondDeposit);
                        break;
                    case 4:
                        hardware.drive.followTrajectorySplineHeading(trajectories.thirdDeposit);
                        break;
                    case 5:
                        hardware.drive.followTrajectorySplineHeading(trajectories.forthDeposit);
                        break;
                    case 6:
                        hardware.drive.followTrajectorySplineHeading(trajectories.fifthDeposit);
                        break;
                }

                outtakeSubsystem.liftToInternalPIDTicks(730);
                outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SPECIMEN);
                outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN);
                if (delay(100))
                {
                    //outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftHighBarPos);

                    if (delay(200))
                    {
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HIGH);
                    }
                    if (delay(300))
                    {
                        outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.SPECIMEN_HIGH);
                    }
                }
                break;
            case DROP:
                if (cycle == 0 ? delay(400) : delay(230))
                {
                    state = cycle == 1 ? autoState.EJECTION_TO_HP : cycle == 5 ? autoState.PARK : autoState.INTAKE;
                    cycle++;
                    resetTimer();
                    break;
                }
                outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                if (cycle == 0 && delay(90))
                    {
                        intakeSubsystem.intakeSlideInternalPID(0);
                    }
                if (delay(120))
                {
                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.STRAIGHT);
                }
                break;
            case PARK:
                if (trajectories.parkTrajectory.isFinished() && delay(1000))
                {
                    state = autoState.IDLE;
                    resetTimer();
                    break;
                }
                if (delay(400))
                {
                    outtakeSubsystem.liftToInternalPIDTicks(0);
                    outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.MIDDLE);
                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER);
                }
                hardware.drive.followTrajectorySplineHeading(trajectories.parkTrajectory);
                break;
            case IDLE: // we idle here duuhhh
                break;
        }

    }
    public void outtakeLiftPresets(boolean isSample, boolean low)
    {
        if (isSample)
        {
            if (low) outtakeSubsystem.liftToInternalPIDTicks(350);
                //outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftLowBucketPos);
            else  outtakeSubsystem.liftToInternalPIDTicks(1655);
            //outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftHighBucketPos);
        }
        else
        {
            if (low)  outtakeSubsystem.liftToInternalPIDTicks(0);
                //outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftLowBarPos);
            else  outtakeSubsystem.liftToInternalPIDTicks(700);
            //outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftHighBarPos);
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
    public void drawSub(Telemetry telemetry, int row, int column)
    {
        int rows = 3;
        int cols = 6;
        char[][] grid = new char[rows][cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                grid[i][j] = '-'; // Fill with default character
            }
        }
        // Fill the specified square
        grid[row][column] = 'X';
        for (int i = 0; i < rows; i++)
        {
            telemetry.addLine(Arrays.toString(grid[row]));
        }
    }

    public boolean delay(double delayTime)
    {
        return (globalTimer - sequenceTimer) > delayTime;
    }
    public boolean before(double delayTime)
    {
        return (globalTimer - sequenceTimer) < delayTime;
    }

    public void resetTimer()
    {
        sequenceTimer = globalTimer;
    }
}
