package org.firstinspires.ftc.teamcode.opmode.auto;


import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTransfer;

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
public class Specimen7AutoNew extends LinearOpMode
{
    boolean armDown;

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

        intakeSubsystem.intakeFilter = IntakeSubsystem.IntakeFilter.SIDE_ONLY;
        resetTimer();

        while (!isStarted())
        {
            hardware.resetCacheHubs();
            outtakeSubsystem.outtakeReads();
            outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);

            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.INTAKE);
            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.TRANSFER_FRONT);
            outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.RIGHT);
            outtakeSubsystem.turretSpinToGains(OuttakeSubsystem.OuttakeTurretState.TRANSFER_FRONT);

            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.READY);
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
            intakeSubsystem.intakeReads(true); // we dont need the color sensor in this auto
            outtakeSubsystem.outtakeReads();

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
            telemetry.addLine();
            telemetry.addData("Pickup state", pickupState);
            telemetry.addData("Distance", intakeSubsystem.getDistance());
            telemetry.addData("Colour", colourValue);

            telemetry.addData("HeadingPosition", headingPosition);
            telemetry.addData("Pose", hardware.drive.getPoseEstimate());
            telemetry.update();
        }
    }
    public void autoSequence()
    {
        switch (state)
        {
            case PRELOAD_DEPOSIT:
                switch (outtakeState)
                {
                    case READY:
                        hardware.drive.followTrajectorySplineHeading(trajectories.preloadTrajectory);
                        outtakeSubsystem.liftToInternalPID(outtakeSubsystem.liftHighBarPos);
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HORIZONTAL);
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN_HIGH);
                        outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SPECIMEN_HIGH);
                        outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.RIGHT);
                        outtakeSubsystem.turretSpinToGains(OuttakeSubsystem.OuttakeTurretState.TRANSFER_FRONT);
                        if ((intakeSubsystem.isDistance(1.7) && delay(200)) || delay(2000))
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
                        colourValue = intakeSubsystem.getColorValue(); // we attempt intake in this case
                        intakeSubsystem.intakeSlideInternalPID(armDown ? 11 : 4);
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
                            resetTimer();
                            break;
                        }
                        break;
                }
                break;
            case SUB_TO_HP:
                hardware.drive.followTrajectorySplineHeading(trajectories.subToHpAndIntake);
                switch (outtakeState)
                {
                    case TRANSFER_START:
                        if (intakeSubsystem.ticksToInchesSlidesMotor(intakeSubsystem.slidePosition) < 2)
                        {
                            intakeClipHoldLogicWithoutPowerCutout(slideTransfer, 10); // this controls the intake slides and the clip
                        } else if (delay(120)) intakeSubsystem.intakeSlideMotorRawControl(-1);

                        outtakeSubsystem.turretSpinTo(OuttakeSubsystem.OuttakeTurretState.TRANSFER_FRONT);
                        if (yPosition + Math.max(0, intakeSubsystem.ticksToInchesSlidesMotor(intakeSubsystem.slidePosition)) + 6.5 < -26 || delay(650))
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
                            intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
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
                break;
            case SUB_INTAKE:
                if (delay(100)) hardware.drive.followTrajectorySplineHeading(trajectories.hpToSubIntake);
                switch (outtakeState)
                {
                    case READY:
                        if (delay(0))
                        {
                            intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);
                            outtakeSubsystem.liftToInternalPID(outtakeSubsystem.liftHighBarPos);
                            if (delay(200))
                            {
                                intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HORIZONTAL);
                                outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SPECIMEN_HIGH);
                                outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN_HIGH);
                                outtakeSubsystem.turretSpinToGains(OuttakeSubsystem.OuttakeTurretState.TRANSFER_FRONT);
                            }
                        }
                        if (trajectories.hpToSubIntake.isFinished() || intakeSubsystem.isDistance(1.7) && delay(220) || delay(2000))
                        {
                            outtakeState = OuttakeState.DROP;
                            outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.INTAKE);
                            internalTimerReset(); // we need to use the internal timer as the trajectory is timed out
                            armDown = false;
                            break;
                        }
                        break;
                    case DROP:
                        colourValue = intakeSubsystem.getColorValue(); // we attempt intake in this case
                        intakeSubsystem.intakeSlideInternalPID(armDown ? 16 : 4);
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                        if (intakeSubsystem.slideOverPosition(2))
                        {
                            armDown = true;
                            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DOWN);
                        }

                        if (colourValue > 900 || delay(2200))
                        {
                            state = autoState.INTAKE_AND_DROP;
                            pickupState = PickupState.TRANSFER_START;
                            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HORIZONTAL);
                            resetTimer();
                            break;
                        }
                        break;
                }
                break;
            case INTAKE_AND_DROP:
                Trajectory pickupTrajectory = null;
                switch (pickupCycle)
                {
                    case 0:
                    case 1:
                        pickupTrajectory = trajectories.subDropAndFirstSample;
                        break;
                    case 2:
                        pickupTrajectory = trajectories.secondSample;
                        break;
                    case 3:
                        pickupTrajectory = trajectories.thirdSample;
                        break;
                }
                if (pickupTrajectory != null) hardware.drive.followTrajectorySplineHeading(pickupTrajectory);

                switch (pickupState)
                {
                    case TRANSFER_START:
                        if (intakeSubsystem.ticksToInchesSlidesMotor(intakeSubsystem.slidePosition) < 2)
                        {
                            intakeClipHoldLogicWithoutPowerCutout(slideTransfer, 10); // this controls the intake slides and the clip
                        } else if (delay(120)) intakeSubsystem.intakeSlideMotorRawControl(-1);

                        outtakeSubsystem.turretSpinToGains(OuttakeSubsystem.OuttakeTurretState.TRANSFER_FRONT);
                        if (yPosition + intakeSubsystem.ticksToInchesSlidesMotor(intakeSubsystem.slidePosition) + 6.5 < -26)
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
                        if (delay(100))
                        {
                            intakeSubsystem.intakeSlideMotorRawControl(0);
                        }
                        if (delay(150))
                        {
                            intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                        }
                        break;
                    case SUB_DROP:
                        intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.AURA_TRANSFER);
                        intakeSubsystem.intakeSlideInternalPID(17);

                        if (delay(100))
                            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.INTAKE);
//                        outtakeSubsystem.liftToInternalPID(0);
                        outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.INTAKE);
                        outtakeSubsystem.turretSpinTo(OuttakeSubsystem.OuttakeTurretState.TRANSFER_FRONT);
                        if (delay(100))
                            outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.RIGHT);
                        if (xPosition > 48 && false)
                        {
                            pickupState = PickupState.SAMPLE_PICKUP;
//                            outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.TRANSFER_FRONT);
                            resetTimer();
                            break;
                        }
                        break;
                    case SAMPLE_PICKUP:
                        colourValue = intakeSubsystem.getColorValue();
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                        outtakeSubsystem.liftToInternalPID(0);
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DOWN);
                        if (delay(20)) intakeSubsystem.intakeSlideInternalPID(18.7);
                        if (intakeSubsystem.slideOverPosition(16.5))
                        {
                            if (pickupCycle > 2)
                                intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.MAX_RIGHT);
                            else
                                intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.MAX_LEFT);
                        } else
                        {
                            if (pickupCycle > 2)
                                intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.RIGHT);
                            else
                                intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.LEFT);
                        }
                        if (delay(1500) || (colourValue > 800 && delay(200)))
                        {
                            pickupState = PickupState.TRANSFER_START;
                            resetTimer();
                            break;
                        }
                        break;
                    case SAMPLE_DROP:
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.INTAKE);
                        outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.INTAKE);
                        if (delay(500))
                        {
                            outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.TRANSFER_FRONT);
                        }
                        if (delay(600))
                        {
                            if (pickupCycle == 3) state = autoState.INTAKE;
                            pickupState = PickupState.SAMPLE_PICKUP;
                            pickupCycle++;
                            resetTimer();
                            break;
                        }
                        break;
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
                        hardware.drive.followTrajectorySplineHeading(trajectories.thirdSampleToIntake);
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
                    case 4:
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
                    case 5:
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
                    case 6:
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
                    case 7:
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
                        outtakeSubsystem.turretSpinToGains(OuttakeSubsystem.OuttakeTurretState.TRANSFER_FRONT);
                        if (cycle == 2)
                        {
                            outtakeSubsystem.liftToInternalPID(0);
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
                            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN_HIGH_BACK_STATIC);
                            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SPECIMEN_HIGH_BACK_STATIC);
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
                if (cycle > 1)
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
                                outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN_HIGH_BACK_STATIC);
                                outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SPECIMEN_HIGH_BACK_STATIC);
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
                    outtakeSubsystem.liftToInternalPID(20);
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
