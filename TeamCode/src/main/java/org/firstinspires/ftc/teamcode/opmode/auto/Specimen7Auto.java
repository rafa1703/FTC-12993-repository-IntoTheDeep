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
import org.firstinspires.ftc.teamcode.opmode.auto.paths.SpecAutoPath;
import org.firstinspires.ftc.teamcode.system.accessory.ToggleUpOrDown;
import org.firstinspires.ftc.teamcode.system.accessory.ToggleUpOrDownCircular;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;
import org.firstinspires.ftc.teamcode.system.vision.CameraHardware;
import org.firstinspires.ftc.teamcode.system.vision.InverseKinematics;

import java.util.Arrays;

@Autonomous(name = "7+0", group = "SPEC")
public class Specimen7Auto extends LinearOpMode
{
    enum autoState {
        PRELOAD_DEPOSIT,
        SUB_INTAKE,
        SUB_TO_HP,
        INTAKE_AND_DROP,
        INTAKE,
        DEPOSIT_DRIVE,
        DROP,
        PARK,
        IDLE
    }
    enum PickupState
    {
        SUB_DROP,
        FIRST_PICKUP,
        FIRST_DROP,
    }
    ElapsedTime GlobalTimer;
    autoState state = autoState.PRELOAD_DEPOSIT;
    GeneralHardware hardware;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    SpecAutoPath trajectories = new SpecAutoPath();
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    CameraHardware cameraHardware;
    InverseKinematics cameraInverseKinematics = new InverseKinematics();
    Pose preloadPose = trajectories.preloadTrajectory.getFinalPose();
    double globalTimer, sequenceTimer, internalTimer, intakeClipTimer, turretTimer;
    int cycle = 0;
    PickupState pickupState = PickupState.SUB_DROP;
    int subIntakeCycle = 0;
    int pickupCycle;
    boolean intakedSpec = false;
    boolean cachedSlideTarget = false;
    double slideCachedTarget;
    boolean startedExtending = false;
    boolean dropped = false;
    boolean intakedSample;
    boolean transferred = false;
    double xPosition, yPosition, headingPosition;
    double headingErrorToEndPose;
    ToggleUpOrDownCircular rowToggle = new ToggleUpOrDownCircular(1, 1, 0, 2);
    ToggleUpOrDownCircular columnToggle = new ToggleUpOrDownCircular(1, 1, 3, 5);
    ToggleUpOrDown angleToggle = new ToggleUpOrDown(1, 1, 18);

    @Override
    public void runOpMode() throws InterruptedException
    {
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.Red, true);
        hardware.drive.setRunMode(MecanumDrive.RunMode.Vector);
        hardware.drive.getLocalizer().setOffSet(trajectories.farStartPose);
        intakeSubsystem = new IntakeSubsystem(hardware);
        outtakeSubsystem = new OuttakeSubsystem(hardware);
        cameraHardware = new CameraHardware(hardware);
        GlobalTimer = new ElapsedTime(System.nanoTime());
        globalTimer = GlobalTimer.milliseconds();

        intakeSubsystem.intakeFilter = IntakeSubsystem.IntakeFilter.SIDE_ONLY;
        resetTimer();

        while (!isStarted())
        {

            intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.HOLD);
            outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SPECIMEN_HIGH);
            outtakeSubsystem.turretSpinTo(0);


            rowToggle.upToggle(gamepad1.dpad_up);
            rowToggle.downToggle(gamepad1.dpad_down, 1);
            columnToggle.upToggle(gamepad1.dpad_right);
            columnToggle.downToggle(gamepad1.dpad_left, 1);
            angleToggle.upToggle(gamepad1.left_bumper);
            angleToggle.downToggle(gamepad1.right_bumper, 1);
            drawSub(telemetry, rowToggle.OffsetTargetPosition, columnToggle.OffsetTargetPosition);
            double angle = angleToggle.OffsetTargetPosition * 5;
            telemetry.addData("Angle: ", angle);
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
            intakeSubsystem.intakeReads(state == autoState.PRELOAD_DEPOSIT ||
                    state == autoState.SUB_INTAKE || state == autoState.INTAKE_AND_DROP); // we dont need the color sensor in this auto
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
            telemetry.addData("Pickup cycle", pickupState);
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

                if (cameraHardware.getLatestResult().isValid() && !cachedSlideTarget)
                {
                    Pose allingPose = preloadPose.plus(new Pose(0, 0, Math.toRadians(cameraHardware.getLatestResult().getTx())));
                    hardware.drive.setTargetPose(allingPose);
                }
                else hardware.drive.setTargetPose(preloadPose);

                if (delay(20))
                {
                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN_HIGH);
                    outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.RIGHT);
                }
                if (dropped)
                {
                    if (hardware.drive.reachedHeading(Math.toRadians(2)))
                    {
                        if (!cachedSlideTarget)
                        {
                            slideCachedTarget = cameraInverseKinematics.distanceToSample(cameraHardware.getLatestResult().getTy()); // cache distance because the slides will get in front of the camera
                            cachedSlideTarget = true;
                        }
                        else if (internalDelay(90)) intakeSubsystem.intakeSlideInternalPID(slideCachedTarget - 6.5);
                    }
                    if(internalDelay(100))
                    {
                        outtakeSubsystem.liftTo(0);
                        turretSpinTo(0, OuttakeSubsystem.OuttakeArmServoState.TRANSFER_FRONT,
                                OuttakeSubsystem.OuttakeWristServoState.TRANSFER_FRONT);
                    }
                    if ((intakeSubsystem.getColorValue() > 800 && internalDelay(150)) || internalDelay(1200))
                    {
                        state = autoState.SUB_TO_HP;
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HALF_TRANSFER);
                        resetTimer();
                        break;
                    }
                }
                else
                {
                    outtakeSubsystem.turretKeepToAngle(90, Math.toDegrees(headingPosition));
                    if (intakeSubsystem.isDistance(2) || trajectories.preloadTrajectory.isFinished())
                    {
                        outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                        dropped = true;
                        internalTimerReset();
                    }
                }
                break;
            case SUB_TO_HP:

                hardware.drive.followTrajectorySplineHeading(trajectories.subToHpAndIntake);
                if (delay(90)) intakeSubsystem.intakeSlideInternalPID(-2);

                if (!transferred)
                {
                    if (delay(200)) // trasnfer + hp deposit position
                    {
                        if (delay(300))
                        {
                            outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                        }
                        if (delay(400))
                        {
                            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.TRANSFER_FINISH);
                            intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                        } else
                        {
                            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.TRANSFER_FRONT);
                        }
                        if (delay(500))
                        {
                            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.HP_DEPOSIT);
                            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.HP_DEPOSIT);
                            outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.UP);
                        }
                        if (delay(600))
                        {
                            transferred = true;
                        }
                    }
                }
                else
                {
                    outtakeSubsystem.turretKeepToAngle(270, headingPosition, true);
                    if (trajectories.subToHpAndIntake.isFinished() && outtakeSubsystem.turretReached())
                    {
                        outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.INTAKE);
                        dropped = true;
                        internalTimerReset();
                    }
                }
                if (internalDelay(200) && dropped)
                {
                    state = autoState.SUB_INTAKE;
                    resetTimer();
                    break;
                }

                break;
            case SUB_INTAKE:
                if (!intakedSpec)
                {
                    outtakeSubsystem.liftToInternalPID(6);
                    if (outtakeSubsystem.liftReached(6))
                    {
                        outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                        intakedSpec = true;
                        resetTimer();
                    }
                }
                else
                {
                    if (delay(90))
                    {
                        hardware.drive.followTrajectorySplineHeading(trajectories.hpToSubIntake);
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN_HIGH);
                        outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SPECIMEN_HIGH);

                        if (intakeSubsystem.isDistance(2) || trajectories.hpToSubIntake.isFinished())
                        {
                            outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                            dropped = true;
                            internalTimerReset();
                        }
                    }
                    if (dropped && internalDelay(100))
                    {
                        state = autoState.INTAKE_AND_DROP;
                        subIntakeCycle++;
                        resetTimer();
                        break;
                    }
                }
                break;
            case INTAKE_AND_DROP:
                hardware.drive.followTrajectorySplineHeading(trajectories.subToSamples);
                switch (pickupState)
                {
                    case SUB_DROP:
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.HP_DEPOSIT);
                        outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.HP_DEPOSIT);
                        outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.UP);
                        if (xPosition > 20)
                        {
                            intakeSubsystem.intakeSlideInternalPID(8);
                            intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.MAX_RIGHT);
                        }
                        if (xPosition > 25)
                        {
                            outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                            pickupState = PickupState.FIRST_PICKUP;
                            pickupCycle++;
                            resetTimer();
                            break;
                        }
                        break;
                    case FIRST_PICKUP:
                        if (delay(100))
                        {
                            intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                        }
                        if (!intakedSample)
                        {
                            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER_FRONT);
                            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.TRANSFER_FRONT);
                            outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.UP);

                            if (intakeSubsystem.getColorValue() > 800)
                            {
                                intakedSample = true;
                                internalTimerReset();
                            }
                        }
                        else
                        {
                            intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);
                            if (internalDelay(90))
                            {
                                intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.TRANSFER_FRONT);
                            }
                            if (internalDelay(220))
                            {
                                outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                            }
                            if (delay(310))
                            {
                                intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                                intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DOWN);
                            }
                            if (delay(400))
                            {
                                outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.HP_DEPOSIT);
                                outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.HP_DEPOSIT);
                                outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.UP);
                            }
                            if (delay(490))
                            {
                                pickupState = PickupState.FIRST_DROP;
                                intakedSample = false;
                                resetTimer();
                                internalTimerReset();
                                break;
                            }
                        }
                        break;
                    case FIRST_DROP:
                        if (!intakedSample)
                        {
                            if (delay(20))
                            {
                                outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                                intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.MAX_RIGHT);
                                intakeSubsystem.intakeSlideInternalPID(8);
                            }
                            if (delay(120))
                            {
                                pickupState = PickupState.FIRST_PICKUP;
                                if (pickupCycle == 2) state = autoState.INTAKE;
                                intakedSample = false;
                                pickupCycle++;
                                resetTimer();
                                internalTimerReset();
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
                switch (cycle)
                {
                    case 2:
                        hardware.drive.followTrajectorySplineHeading(trajectories.firstIntake);
                        if (trajectories.firstIntake.isFinished() && hardware.drive.stopped())
                        {
                            outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                            state = autoState.DEPOSIT_DRIVE;
                            resetTimer();
                        }
                        break;
                    case 3:
                        hardware.drive.followTrajectorySplineHeading(trajectories.secondIntake);
                        if (delay(40))
                        {
                            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DOWN);
                            intakeSubsystem.intakeSlideInternalPID(IntakeSubsystem.slideExtensionLimit);
                            if (intakeSubsystem.getColorValue() > 800)
                            {
                                intakeSubsystem.intakeSlideInternalPID(-2);
                                state = autoState.DEPOSIT_DRIVE;
                                break;
                            }
                        }
                    case 4:
                        hardware.drive.followTrajectorySplineHeading(trajectories.thirdIntake);
                        if (delay(40))
                        {
                            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DOWN);
                            intakeSubsystem.intakeSlideInternalPID(IntakeSubsystem.slideExtensionLimit);
                            if (intakeSubsystem.getColorValue() > 800)
                            {
                                intakeSubsystem.intakeSlideInternalPID(-2);
                                state = autoState.DEPOSIT_DRIVE;
                                break;
                            }
                        }
                    case 5:
                        hardware.drive.followTrajectorySplineHeading(trajectories.forthIntake);
                        if (delay(40))
                        {
                            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DOWN);
                            intakeSubsystem.intakeSlideInternalPID(IntakeSubsystem.slideExtensionLimit);
                            if (intakeSubsystem.getColorValue() > 800)
                            {
                                intakeSubsystem.intakeSlideInternalPID(-2);
                                state = autoState.DEPOSIT_DRIVE;
                                break;
                            }
                        }
                    case 6:
                        hardware.drive.followTrajectorySplineHeading(trajectories.fifthIntake);
                        if (delay(40))
                        {
                            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DOWN);
                            intakeSubsystem.intakeSlideInternalPID(IntakeSubsystem.slideExtensionLimit);
                            if (intakeSubsystem.getColorValue() > 800)
                            {
                                intakeSubsystem.intakeSlideInternalPID(-2);
                                state = autoState.DEPOSIT_DRIVE;
                                break;
                            }
                        }
                        break;
                }

                if (delay(40))
                {
                    if (delay(140))
                    {
                        outtakeSubsystem.turretSpinTo(0);
                        if (cycle == 2)
                        {
                            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.INTAKE);
                            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.INTAKE);
                            outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.RIGHT);
                        } else
                        {
                            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER_BACK);
                            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.TRANSFER_BACK);
                            outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.UP);
                        }
                    }
                }
                break;
            case DEPOSIT_DRIVE:
                if (
                        ((cycle == 2 && trajectories.firstDepositWhileTurning.isFinished()) ||
                                (cycle == 3 && trajectories.secondDeposit.isFinished()) ||
                                (cycle == 4 && trajectories.thirdDeposit.isFinished()) ||
                                (cycle == 5 && trajectories.forthDeposit.isFinished()) ||
                                (cycle == 6 && trajectories.fifthDeposit.isFinished())) ||
                                (delay(300) && intakeSubsystem.isDistance(2))
                )
                {
                    state = autoState.DROP;
                    resetTimer();
                    break;
                }
                switch (cycle)
                {
                    case 2:
                        hardware.drive.followTrajectorySplineHeading(trajectories.firstDepositWhileTurning);
                        if (delay(90))
                        {
                            outtakeSubsystem.liftToInternalPID(outtakeSubsystem.liftHighBarPos);
                        }
                        if (delay(130))
                        {
                            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN_HIGH_BACK);
                            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SPECIMEN_HIGH_BACK);
                        }
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
                if (cycle > 2)
                {
                    intakeSubsystem.intakeSlideInternalPID(-2);
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.TRANSFER_BACK);
                    if (delay(40) && intakeSubsystem.isSlidesAtBase())
                    {
                        if (!transferred)
                        {
                            outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                            transferred = true;
                            internalTimerReset();
                        }
                        else
                        {
                            if (internalDelay(90))
                            {
                                intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                                intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.TRANSFER_FINISH);
                            }
                            if (internalDelay(160))
                            {
                                outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN_HIGH_BACK);
                                outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SPECIMEN_HIGH_BACK);
                                outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.DOWN);
                            }
                        }
                    }
                }
                break;
            case DROP:
                if (cycle < 2)
                {
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                    if (delay(60))
                    {
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.STRAIGHT);
                        state = cycle == 0 ? autoState.SUB_INTAKE : autoState.INTAKE_AND_DROP;
                        cycle++;
                        resetTimer();
                        break;
                    }
                }
                else
                {
                    outtakeSubsystem.liftToInternalPID(13);
                    if (outtakeSubsystem.liftReached(13))
                    {
                        outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                        state = autoState.INTAKE;
                        cycle++;
                        resetTimer();
                        break;
                    }
                }

                break;
            case PARK:
                if (trajectories.parkTrajectory.isFinished())
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
    public void turretSpinTo(double targetAngle, OuttakeSubsystem.OuttakeArmServoState armState, OuttakeSubsystem.OuttakeWristServoState wristState)
    { // this currently has power cutoff
        if (outtakeSubsystem.turretReached(targetAngle))
        {
            if (globalTimer - turretTimer > 150)
            {
                if (armState != null) // i just did this if i don't really want to pass a state the correct would be to pass default in the state
                    outtakeSubsystem.armState(armState);
                if (wristState != null)
                    outtakeSubsystem.wristState(wristState);
                outtakeSubsystem.turretRawControl(0);
            }
            else
            {
                outtakeSubsystem.turretSpinToCorrected(targetAngle);
            }
        }
        else
        {
            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPIN);
            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SPIN);
            outtakeSubsystem.turretSpinToCorrected(targetAngle); // we
            globalTimer = turretTimer;
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

    public boolean internalDelay(double delayTime)
    {
        return globalTimer - internalTimer > delayTime;
    }
    public void internalTimerReset()
    {
        internalTimer = globalTimer;
    }
}
