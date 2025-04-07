package org.firstinspires.ftc.teamcode.opmode.auto;


import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideExtensionLimit;
import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTransfer;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
public class Specimen7AutoNew extends LinearOpMode
{
    boolean armDown;

    enum autoState {
        PRELOAD_DEPOSIT,
        SUB_INTAKE,
        SUB_TO_HP,
        PRELOADS_INTAKE,
        PRELOADS_DROP,
        INTAKE,
        DEPOSIT_DRIVE,
        DROP,
        PARK,
        IDLE
    }
    enum OuttakeState
    {
        INTAKE,
        TRANSFER_START,
        TRANSFER_END,
        READY,
        DROP
    }
    enum PickupState
    {
        TRANSFER_START,
        TRANSFER_END,
        SAMPLE_THROW,
        SUB_DROP,
        SAMPLE_PICKUP,
        SAMPLE_DROP,
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
    OuttakeState outtakeState = OuttakeState.READY;
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
    double colourValue;
    ToggleUpOrDownCircular rowToggle = new ToggleUpOrDownCircular(1, 1, 0, 2);
    ToggleUpOrDownCircular columnToggle = new ToggleUpOrDownCircular(1, 1, 3, 5);
    ToggleUpOrDown angleToggle = new ToggleUpOrDown(1, 1, 18);

    @Override
    public void runOpMode() throws InterruptedException
    {
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.RED, true);
        hardware.drive.setRunMode(MecanumDrive.RunMode.Vector);
        hardware.drive.getLocalizer().setOffSet(trajectories.farStartPose);
        intakeSubsystem = new IntakeSubsystem(hardware);
        outtakeSubsystem = new OuttakeSubsystem(hardware);
        cameraHardware = new CameraHardware(hardware);
        GlobalTimer = new ElapsedTime(System.nanoTime());
        globalTimer = GlobalTimer.milliseconds();
        hardware.limelight.close();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intakeSubsystem.intakeFilter = IntakeSubsystem.IntakeFilter.SIDE_ONLY;
        resetTimer();
        cameraHardware.start();
        while (!isStarted())
        {
            hardware.resetCacheHubs();
            outtakeSubsystem.outtakeReads();
            outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);

            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.TRANSFER_FRONT);
            outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.RIGHT);
            outtakeSubsystem.turretSpinToGains(OuttakeSubsystem.OuttakeTurretState.TRANSFER_FRONT);

            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.IN);
            intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);
            intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
//
//            rowToggle.upToggle(gamepad1.dpad_up);
//            rowToggle.downToggle(gamepad1.dpad_down, 1);
//            columnToggle.upToggle(gamepad1.dpad_right);
//            columnToggle.downToggle(gamepad1.dpad_left, 1);
//            angleToggle.upToggle(gamepad1.left_bumper);
//            angleToggle.downToggle(gamepad1.right_bumper, 1);
//            drawSub(telemetry, rowToggle.OffsetTargetPosition, columnToggle.OffsetTargetPosition);
//            double angle = angleToggle.OffsetTargetPosition * 5;
//            telemetry.addData("Angle: ", angle);
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();
            Pose poseEstimate = hardware.drive.getPoseEstimate();
            telemetry.addData("Turret", outtakeSubsystem.turretTicksToAngle(outtakeSubsystem.turretIncrementalPosition));
            telemetry.addData("Turret initial offset", outtakeSubsystem.initialOffsetPosition);
            DashboardUtil.drawRobot(fieldOverlay, poseEstimate.toPose2d(), true, "black");
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
            intakeSubsystem.intakeReads(true, state == autoState.PRELOAD_DEPOSIT || state == autoState.SUB_INTAKE || state == autoState.INTAKE || state == autoState.PRELOADS_INTAKE); // we dont need the color sensor in this auto
            outtakeSubsystem.outtakeReads(false);

            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            autoSequence();
            hardware.update();
            Pose poseEstimate = hardware.drive.getPoseEstimate();
            xPosition = poseEstimate.getX();
            yPosition = poseEstimate.getY();
            headingPosition = poseEstimate.getHeading();

            DashboardUtil.drawRobot(fieldOverlay, poseEstimate.toPose2d(), true, "black");
            //DashboardUtil.drawRobot(fieldOverlay, hardware.drive.getPredictedPoseEstimate().toPose2d(), true, "black");
            DashboardUtil.drawCurve(fieldOverlay, hardware.drive.trajectoryFollowing);
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("State", state);
            telemetry.addData("Outtake State", outtakeState);
            telemetry.addData("Pickup cycle", pickupCycle);
            telemetry.addData("Cycle", cycle);
            telemetry.addData("Distance", intakeSubsystem.getDistance());
            telemetry.addLine();
//            telemetry.addData("Distance", intakeSubsystem.getDistance());
//            telemetry.addData("Colour", colourValue);
//
//            telemetry.addData("HeadingPosition", headingPosition);
            telemetry.addData("Pose", hardware.drive.getPoseEstimate());
            telemetry.update();
        }
    }
    public void autoSequence()
    {
        switch (state)
        {
            case PRELOAD_DEPOSIT:
                preloadOuttakeState();
                break;
            case SUB_TO_HP:
                hardware.drive.followTrajectorySplineHeading(trajectories.subToHpAndIntake);
                subToHpOuttakeState();
                break;
            case SUB_INTAKE:
                if (delay(100)) hardware.drive.followTrajectorySplineHeading(trajectories.hpToSubIntake);
                subIntakeOuttakeState();
                break;
            case PRELOADS_INTAKE:
                Trajectory pickupTrajectory = null;
                switch (pickupCycle)
                {
                    case 1:
                        pickupTrajectory = trajectories.firstSample;
                        break;
                    case 2:
                        if (delay(150))
                            pickupTrajectory = trajectories.secondSample;
                        break;
                    case 3:
                        pickupTrajectory = trajectories.thirdSample;
                        break;
                }
                if (pickupTrajectory != null) hardware.drive.followTrajectorySplineHeading(pickupTrajectory);

                colourValue = intakeSubsystem.getColourValue();

                intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                outtakeSubsystem.liftToInternalPID(0);
                intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DOWN);

                if (delay(20)) extendIntakeSlides(pickupCycle);
//                intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.MAX_RIGHT);
                if (delay(60)) intakeSubsystem.intakeTurretSetPos(0.16);
                outtakeSubsystem.turretSpinTo(OuttakeSubsystem.OuttakeTurretState.HP_DROP_AUTO);
                outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.STRAIGHT);
                if (delay(1500) || (colourValue > 200 && delay(500)))
                {
                    state = autoState.PRELOADS_DROP;
                    pickupState = PickupState.SAMPLE_THROW;
                    resetTimer();
                    break;
                }

                break;
            case PRELOADS_DROP:
                Trajectory preloadDropTrajectory = null;
                switch (pickupCycle)
                {
                    case 0:
                        preloadDropTrajectory = trajectories.subDrop;
                        break;
                    case 1:
                        preloadDropTrajectory = trajectories.firstSample;
                        break;
                    case 2:
                        preloadDropTrajectory = trajectories.secondSample;
                        break;
                    case 3:
                        preloadDropTrajectory = trajectories.thirdSample;
                        break;
                }
                if (preloadDropTrajectory != null) hardware.drive.followTrajectorySplineHeading(preloadDropTrajectory);

                preloadsDropOuttakeState();
                break;
                case INTAKE:
                if ((intakedSpec && delay(190)))
                {
                    state =  autoState.DEPOSIT_DRIVE;
                    intakedSpec = false;
                    resetTimer();
                    break;
                }
                if (cycle == 2) outtakeSubsystem.turretSpinTo(OuttakeSubsystem.OuttakeTurretState.TRANSFER_FRONT);
                else
                {
                    intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                    outtakeSubsystem.turretSpinTo(OuttakeSubsystem.OuttakeTurretState.TRANSFER_BACK);
                }
                intakeTrajectory(cycle);
                if (delay(140))
                {
                    outtakeSubsystem.liftToInternalPID(0);
                    if (cycle == 2)
                    {
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.INTAKE);
                        outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.INTAKE);
                        outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.RIGHT);

                    } else
                    {
                        intakeSubsystem.intakeSlideInternalPID(slideExtensionLimit);

                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER_BACK);
                        outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.TRANSFER_BACK);
                        outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.UP);
                    }
                }

                break;
            case DEPOSIT_DRIVE:
                if (
                        ((cycle == 2 && trajectories.firstDepositWhileTurning.isFinished()) ||
                                (cycle == 3 && trajectories.secondDeposit.isFinished()) ||
                                (cycle == 4 && trajectories.thirdDeposit.isFinished()) ||
                                (cycle == 5 && trajectories.forthDeposit.isFinished()) ||
                                (cycle == 6 && trajectories.fifthDeposit.isFinished()) ||
                                (cycle == 7 && trajectories.sixthDeposit.isFinished())
                        ) && (internalDelay(750) && transferred)
                )
                {
                    state = autoState.DROP;
                    resetTimer();
                    break;
                }
                depositTrajectory(cycle);

                if (!transferred)
                {
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.TRANSFER_BACK);
                    intakeSubsystem.intakeSlideMotorRawControl(-1);
                }
                if (internalDelay(160) && transferred) // this is here so it runs after the slides go out...
                    outtakeSubsystem.turretSpinTo(OuttakeSubsystem.OuttakeTurretState.SPEC_DEPOSIT_BACK);
                if (cycle == 2 || (delay(400) && intakeSubsystem.isSlidesAtBase()))
                {
                    outtakeSubsystem.liftToInternalPID(0);
                    if (!transferred)
                    {
                        outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                        transferred = true;
                        internalTimerReset();
                    }
                    else
                    {
                        if (internalDelay(200)) intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.OFF);
                        else if (internalDelay(130)) intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                        if (internalDelay(160))
                        {
                            if (cycle != 2 || internalDelay(500))
                            {
                                if (cycle > 4) intakeSubsystem.intakeSlideInternalPID(14);
                                else
                                    intakeSubsystem.intakeSlideInternalPID(30);
                            }
//                            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN_HIGH_AUTO_READY); // FIXME: BIG CHUNGUS IS FAT
                            outtakeSubsystem.wristSetPos(0.7);
                            outtakeSubsystem.liftToInternalPID(10);
                            outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.LEFT);
                            intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                            if (internalDelay(250))
                            {
                                intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HALF_DOWN);
                                intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);
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
                        state = cycle == 0 ? autoState.SUB_INTAKE : autoState.PRELOADS_INTAKE;
                        cycle++;
                        resetTimer();
                        break;
                    }
                }
                else
                {
                    intakeSubsystem.intakeSlideInternalPID(slideExtensionLimit);
                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN_HIGH_AUTO_SCORE);
                    outtakeSubsystem.liftToInternalPID(0);
                    outtakeSubsystem.turretSpinTo(OuttakeSubsystem.OuttakeTurretState.SPEC_DEPOSIT_BACK);
                    if (delay(300)) outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.INTAKE);
                    if (delay(400))
                    {
                        state = autoState.INTAKE;
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER_BACK);
                        outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.TRANSFER_BACK);
//                        outtakeSubsystem.turretSpinTo(OuttakeSubsystem.OuttakeTurretState.TRANSFER_BACK);
                        cycle++;
                        transferred = false;
                        intakedSpec = false;
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

    private void preloadOuttakeState()
    {
        switch (outtakeState)
        {
            case READY:
                hardware.drive.followTrajectorySplineHeading(trajectories.preloadTrajectory);
                intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HORIZONTAL);

                outtakeSubsystem.armSetPos(1);
                outtakeSubsystem.wristSetPos(0.483);
                outtakeSubsystem.liftToInternalPID(outtakeSubsystem.liftHighBarPos + 4);
                outtakeSubsystem.turretSpinToGains(OuttakeSubsystem.OuttakeTurretState.TRANSFER_FRONT);
                intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);

                if ((intakeSubsystem.isDistance() && delay(200)) || delay(2000))
                {
                    outtakeState = OuttakeState.DROP;
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.TRANSFER_FRONT);
                    intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                    intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                    resetTimer();
                    break;
                }
                break;
            case DROP:
                colourValue = intakeSubsystem.getColourValue(); // we attempt intake in this case
                intakeSubsystem.intakeSlideInternalPID(armDown ? 11 : 4);
                outtakeSubsystem.turretSpinToGains(OuttakeSubsystem.OuttakeTurretState.TRANSFER_FRONT);
                if (intakeSubsystem.slideOverPosition(2))
                {
                    armDown = true;
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DOWN);
                }

                if ((colourValue >= 850) || delay(1300))
                {
                    state = autoState.SUB_TO_HP;
                    outtakeState = OuttakeState.TRANSFER_START;
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HORIZONTAL);
                    cycle++;
                    resetTimer();
                    break;
                }
                break;
        }
    }

    private void subToHpOuttakeState()
    {
        switch (outtakeState)
        {
            case TRANSFER_START:
                if (intakeSubsystem.ticksToInchesSlidesMotor(intakeSubsystem.slidePosition) < 2)
                {
                    intakeClipHoldLogicWithoutPowerCutout(slideTransfer, 10); // this controls the intake slides and the clip
                } else if (delay(120)) intakeSubsystem.intakeSlideMotorRawControl(-1);

                outtakeSubsystem.turretSpinTo(OuttakeSubsystem.OuttakeTurretState.TRANSFER_FRONT);
                if (canSlidesComeBackWithoutHitting() || delay(650))
                {
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HP_DEPOSIT);
                    intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.HP_DEPOSIT);
                }
                if (delay(400))
                {
                    outtakeSubsystem.liftToInternalPID(0);
                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.INTAKE);
                    outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.INTAKE);
                    outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.RIGHT);
                }
                if (intakeSubsystem.isSlidesAtBase() && delay(550) || delay(2000))
                {
                    outtakeState = OuttakeState.READY; // drop to hp from the side, skipping transfering
                    resetTimer();
                    break;
                }
                break;
            case READY:
                outtakeSubsystem.liftToInternalPID(0);
                outtakeSubsystem.turretSpinTo(OuttakeSubsystem.OuttakeTurretState.TRANSFER_FRONT);
                outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.INTAKE);
                if (trajectories.subToHpAndIntake.isFinished() && delay(200))
                {
                    outtakeState = OuttakeState.INTAKE;
                    intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.HP_REVERSE);
                    resetTimer();
                    break;
                }
                break;
            case INTAKE:
                if (delay(300)) outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);

                if (delay(450))
                {
                    state = autoState.SUB_INTAKE; // we change the state here
                    outtakeState = OuttakeState.READY;
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HORIZONTAL);
                    resetTimer();
                    break;
                }
                break;
        }
    }

    private void subIntakeOuttakeState()
    {
        switch (outtakeState)
        {
            case READY:
                if (delay(0))
                {
                    intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);
                    outtakeSubsystem.liftToInternalPID(outtakeSubsystem.liftHighBarPos + 4);
                    if (delay(200))
                    {
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HORIZONTAL);
                        outtakeSubsystem.armSetPos(1);
                        outtakeSubsystem.wristSetPos(0.483);
                        outtakeSubsystem.turretSpinToGains(OuttakeSubsystem.OuttakeTurretState.TRANSFER_FRONT);
                    }
                }
                if (trajectories.hpToSubIntake.isFinished() || intakeSubsystem.isDistance() && delay(220) || delay(2000))
                {
                    outtakeState = OuttakeState.DROP;
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.INTAKE);
                    internalTimerReset(); // we need to use the internal timer as the trajectory is timed out
                    armDown = false;
                    break;
                }
                break;
            case DROP:
                colourValue = intakeSubsystem.getColourValue(); // we attempt intake in this case
                intakeSubsystem.intakeSlideInternalPID(armDown ? 16 : 4);
                intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                if (intakeSubsystem.slideOverPosition(2))
                {
                    armDown = true;
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DOWN);
                }

                if (colourValue > 900 || delay(2200))
                {
                    state = autoState.PRELOADS_DROP;
                    pickupState = PickupState.TRANSFER_START;
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HORIZONTAL);
                    intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                    cycle++;
                    resetTimer();
                    break;
                }
                break;
        }
    }

    private void depositTrajectory(int cycle)
    {
        switch (cycle)
        {
            case 2:
                if (delay(90))
                {
                    hardware.drive.followTrajectorySplineHeading(trajectories.firstDepositWhileTurning);
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
            case 7:
                hardware.drive.followTrajectorySplineHeading(trajectories.sixthDeposit);
                break;
        }
    }

    private void intakeTrajectory(int cycle)
    {
        switch (cycle)
        {
            case 2:
                if(delay(1000))
                    hardware.drive.followTrajectorySplineHeading(trajectories.thirdSampleToIntake);
                else hardware.drive.followTrajectorySplineHeading(trajectories.letsNotCutOurHPHands);
                if (trajectories.thirdSampleToIntake.isFinished() && hardware.drive.stopped())
                {
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                    state = autoState.DEPOSIT_DRIVE;
                    intakedSpec = true;
                    resetTimer();
                }
                break;
            case 3:
                hardware.drive.followTrajectorySplineHeading(trajectories.firstIntake);
                if (delay(100))
                {
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DOWN);
                    intakeSubsystem.intakeSlideInternalPID(IntakeSubsystem.slideExtensionLimit);
                    if (trajectories.firstIntake.isFinished() && hardware.drive.stopped())
                    {
                        intakeSubsystem.intakeSlideMotorRawControl(-1);
                        intakedSpec = true;
                        state = autoState.DEPOSIT_DRIVE;
                        break;
                    }
                }
                break;
            case 4:
                hardware.drive.followTrajectorySplineHeading(trajectories.secondIntake);
                if (delay(40))
                {
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DOWN);
                    intakeSubsystem.intakeSlideInternalPID(IntakeSubsystem.slideExtensionLimit);
                    if (trajectories.secondIntake.isFinished() && hardware.drive.stopped())
                    {
                        intakeSubsystem.intakeSlideMotorRawControl(-1);
                        intakedSpec = true;
                        state = autoState.DEPOSIT_DRIVE;
                        break;
                    }
                }
            case 5:
                hardware.drive.followTrajectorySplineHeading(trajectories.thirdIntake);
                if (delay(40))
                {
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DOWN);
                    intakeSubsystem.intakeSlideInternalPID(IntakeSubsystem.slideExtensionLimit);
                    if (trajectories.thirdIntake.isFinished() && hardware.drive.stopped())
                    {
                        intakeSubsystem.intakeSlideMotorRawControl(-1);
                        intakedSpec = true;
                        state = autoState.DEPOSIT_DRIVE;
                        break;
                    }
                }
            case 6:
                hardware.drive.followTrajectorySplineHeading(trajectories.forthIntake);
                if (delay(40))
                {
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DOWN);
                    intakeSubsystem.intakeSlideInternalPID(IntakeSubsystem.slideExtensionLimit);
                    if (trajectories.forthIntake.isFinished() && hardware.drive.stopped())
                    {
                        intakeSubsystem.intakeSlideMotorRawControl(-1);
                        intakedSpec = true;
                        state = autoState.DEPOSIT_DRIVE;
                        break;
                    }
                }
            case 7:
                hardware.drive.followTrajectorySplineHeading(trajectories.fifthIntake);
                if (delay(40))
                {
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DOWN);
                    intakeSubsystem.intakeSlideInternalPID(IntakeSubsystem.slideExtensionLimit);
                    if (trajectories.fifthIntake.isFinished() && hardware.drive.stopped())
                    {
                        intakeSubsystem.intakeSlideMotorRawControl(-1);
                        intakedSpec = true;
                        state = autoState.DEPOSIT_DRIVE;
                        break;
                    }
                }
                break;
        }

    }

    private void preloadsDropOuttakeState()
    {
        switch (pickupState)
        {
            case SAMPLE_THROW:
                outtakeSubsystem.turretSpinTo(OuttakeSubsystem.OuttakeTurretState.HP_DROP_AUTO);

                intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.REALLY_SICK_THROW);
                if (delay(20)) intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.AUTO_REALLY_SICK_THROW);

                if (intakeSubsystem.ticksToInchesSlidesMotor(intakeSubsystem.slidePosition) < 2)
                {
                    intakeSubsystem.intakeSlideInternalPID(0); // this controls the intake slides and the clip
                } else if (delay(170)) intakeSubsystem.intakeSlideMotorRawControl(-0.8);

                if (intakeSubsystem.getSlidePositionIn() < 11) intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                if (intakeSubsystem.getSlidePositionIn() < 1)
                {
                    state = pickupCycle == 3 ? autoState.INTAKE : autoState.PRELOADS_INTAKE;
                    pickupState = PickupState.SAMPLE_PICKUP;
                    if (pickupCycle != 3) extendIntakeSlides(pickupCycle);
                    else
                    {
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.INTAKE);
                        outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.INTAKE);
                        outtakeSubsystem.turretSpinTo(OuttakeSubsystem.OuttakeTurretState.TRANSFER_FRONT);
                    }
                    pickupCycle++;
                    resetTimer();
                    break;
                }
                break;
            case TRANSFER_START:
                if (intakeSubsystem.ticksToInchesSlidesMotor(intakeSubsystem.slidePosition) < 2)
                {
                    intakeSubsystem.intakeSlideInternalPID(0); // this controls the intake slides and the clip
                } else if (delay(120)) intakeSubsystem.intakeSlideMotorRawControl(-1);

                outtakeSubsystem.turretSpinToGains(OuttakeSubsystem.OuttakeTurretState.TRANSFER_FRONT);
                if (canSlidesComeBackWithoutHitting())
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.TRANSFER_FRONT);
                if (delay(200))
                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER_FRONT);
                intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);
                if (yPosition < -35)
                {
                    if (delay(50))
                    {
                        outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.TRANSFER_FRONT);
                        outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.DOWN);
                        outtakeSubsystem.liftToInternalPID(0);
                    }
                }
                if (intakeSubsystem.isSlidesAtBase() && outtakeSubsystem.liftAtBase() && delay(300) || delay(900))
                {
                    pickupState = PickupState.TRANSFER_END;
                    resetTimer();
                    break;
                }
                break;
            case TRANSFER_END:
                if (delay(400))
                {
                    pickupState = PickupState.SAMPLE_DROP;
                    intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.OFF);
                    intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.HOLD);
                    intakeSubsystem.intakeSlideMotorRawControl(0);
                    outtakeSubsystem.liftMotorRawControl(0);
                    resetTimer();
                    break;
                }
                if (delay(80))
                {
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                }
                if (delay(230))
                {
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DOWN);
                    if (pickupCycle != 3)
                        extendIntakeSlides(pickupCycle);
                    else intakeSubsystem.intakeSlideInternalPID(0);
                }
                else if (delay(100))
                {
                    intakeSubsystem.intakeSlideMotorRawControl(0);
                }
                if (delay(150))
                {
                    intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                }
                break;
            case SAMPLE_DROP:
                if (pickupCycle != 3)
                {
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DOWN);
                    extendIntakeSlides(pickupCycle);
                }
                else
                {
                    intakeSubsystem.intakeSlideInternalPID(0);
                    if (delay(500))
                    {
                        outtakeSubsystem.turretSpinTo(OuttakeSubsystem.OuttakeTurretState.TRANSFER_FRONT);
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.INTAKE);
                        outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.INTAKE);
                    }
                }
                if (delay(300)) intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.MAX_RIGHT);
                else intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);
                outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.INTAKE);
                outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.INTAKE);
                if (delay(570))
                {
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.TRANSFER_FRONT);
                }
                if (delay(600))
                {
                    state = pickupCycle == 3 ? autoState.INTAKE : autoState.PRELOADS_INTAKE;
                    pickupState = PickupState.SAMPLE_PICKUP;
                    pickupCycle++;
                    resetTimer();
                    break;
                }
                break;
        }
    }

    private boolean canSlidesComeBackWithoutHitting()
    {
        return yPosition + Math.max(0, intakeSubsystem.ticksToInchesSlidesMotor(intakeSubsystem.slidePosition)) + 6.5 < -26;
    }

    public void intakeClipHoldLogicWithoutPowerCutout(double slideToPosition, int closeThreshold){
        if (intakeSubsystem.slidePosition < closeThreshold) {
            if (globalTimer - intakeClipTimer > 100){
                intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.HOLD); // turn the intake slide pid running to pos off to save battery draw
                intakeSubsystem.intakeSlideInternalPID(0);
            } else {
                intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.HOLD); // turn the intake slide pid running to pos off to save battery draw
                intakeSubsystem.intakeSlideInternalPID(slideToPosition);
            }
        } else {
            intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN); // this might break something when as the intake slides won't go in, but stops jitter
            intakeSubsystem.intakeSlideInternalPID(slideToPosition);
            intakeClipTimer = globalTimer;
        }
    }
    public void extendIntakeSlides(int cycle)
    {
        switch (cycle)
        {
            case 0:
            case 1:
                intakeSubsystem.intakeSlideInternalPID(27);
                break;
            case 2:
                intakeSubsystem.intakeSlideInternalPID(28);
                break;
            case 3:
                intakeSubsystem.intakeSlideInternalPID(30);
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
    public boolean isBetweenAngle(double angle, double min, double max)
    {
        return angle <= max && angle >= min;
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
