package org.firstinspires.ftc.teamcode.opmode.auto;


import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideExtensionLimit;
import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTransfer;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.gvf.MecanumDrive;
import org.firstinspires.ftc.teamcode.gvf.trajectories.Trajectory;
import org.firstinspires.ftc.teamcode.gvf.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.firstinspires.ftc.teamcode.gvf.utils.Vector;
import org.firstinspires.ftc.teamcode.opmode.auto.paths.SpecAutoPath;
import org.firstinspires.ftc.teamcode.system.accessory.LoopTime;
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
        COLOUR_CHECK,
        WRONG_COLOUR,
        TRANSFER_START,
        TRANSFER_END,
        READY,
        DROP
    }
    enum PickupState
    {
        COLOUR_CHECK, // dodgy edge but worlds tomorrow lmao
        WRONG_COLOUR,
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
    LoopTime loopTime = new LoopTime();
    Pose samplePose = null;
    @Override
    public void runOpMode() throws InterruptedException
    {
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.RED, true);
        hardware.drive.setRunMode(MecanumDrive.RunMode.Vector);
        hardware.drive.getLocalizer().setOffSet(trajectories.farStartPose);
        intakeSubsystem = new IntakeSubsystem(hardware);
        outtakeSubsystem = new OuttakeSubsystem(hardware);
        cameraHardware = new CameraHardware(hardware);
        cameraHardware.pipelineSwitch(CameraHardware.PipelineType.RED);
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

            outtakeSubsystem.armSetPos(1);
            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SPECIMEN_AUTO_PRELOADS);
            outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.RIGHT);
            outtakeSubsystem.turretSpinToGains(OuttakeSubsystem.OuttakeTurretState.TRANSFER_FRONT);
            outtakeSubsystem.lockServoState(OuttakeSubsystem.OuttakeLockServoState.OPEN);

            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.IN);
            intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);
            intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);


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
//        if (samplePose == null) throw new RuntimeException("Ling-Ling fault");
        globalTimer = GlobalTimer.milliseconds();
        resetTimer();
        hardware.resetCacheHubs();
        while (opModeIsActive())
        {
            globalTimer = GlobalTimer.milliseconds();
            intakeSubsystem.intakeReads(true, state == autoState.PRELOAD_DEPOSIT || state == autoState.SUB_INTAKE); // we dont need the color sensor in this auto
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
            loopTime.updateLoopTime(telemetry);
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
                        if (delay(400))
                            pickupTrajectory = trajectories.secondSample;
                        break;
                    case 3:
                        if (delay(100))
                            pickupTrajectory = trajectories.thirdSample;
                        break;
                }
                if (pickupTrajectory != null) hardware.drive.followTrajectorySplineHeading(pickupTrajectory);

                colourValue = intakeSubsystem.getColourValue();

                intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                outtakeSubsystem.liftToInternalPID(0);
                intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.EXTENDO_DOWN);
                intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                if (delay(20)) extendIntakeSlides(pickupCycle);
                if (delay(370)) intakeSubsystem.intakeTurretSetPos(0.16);
                if (pickupCycle == 1) outtakeSubsystem.wristSetPos(0.6);
                else outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.INTAKE);
                outtakeSubsystem.turretSpinTo(OuttakeSubsystem.OuttakeTurretState.HP_DROP_AUTO);
                outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.STRAIGHT);
                if (delay(1200) || (colourValue > 150 && delay(500)))
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
                        preloadDropTrajectory = trajectories.firstSampleDrop;
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
                if (intakedSpec && (internalDelay(800) || cycle == 2))
                {
                    state =  autoState.DEPOSIT_DRIVE;
                    intakedSpec = false;
                    resetTimer();
                    internalTimerReset();
                    break;
                }
                Trajectory intakeTrajectory = null;
                switch (cycle)
                {
                    case 2:
                        if(delay(1150))
                            intakeTrajectory = trajectories.thirdSampleToIntake;
                        else intakeTrajectory = trajectories.letsNotCutOurHPHands;
                        break;
                    case 3:
                        intakeTrajectory = trajectories.firstIntake;
                        break;
                    case 4:
                        intakeTrajectory = trajectories.secondIntake;
                        break;
                    case 5:
                        intakeTrajectory = trajectories.thirdIntake;
                        break;
                    case 6:
                        intakeTrajectory = trajectories.forthIntake;
                        break;
                    case 7:
                        intakeTrajectory = trajectories.fifthIntake;
                        break;
                }
                if (intakeTrajectory != null) hardware.drive.followTrajectorySplineHeading(intakeTrajectory);
                if (!intakedSpec)
                {
                    if (cycle == 2) // pickup from wall cycle
                    {
                        outtakeSubsystem.turretSpinTo(OuttakeSubsystem.OuttakeTurretState.TRANSFER_FRONT);
                        if (delay(140))
                        {
                            outtakeSubsystem.liftToInternalPID(0);
                            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.INTAKE);
                            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.INTAKE);
                            outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.RIGHT);
                        }
                        if (trajectories.thirdSampleToIntake.isFinished()) {
                            outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                            intakedSpec = true;
                            internalTimerReset();
                            break;
                        }
                    }
                    else if (intakeTrajectory != null)
                    {
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);

                        if (delay(40))
                        {
                            intakeSubsystem.armSetPos(0.96);
//                            intakeSubsystem.intakeTurretSetPos(0.496);
//                            intakeSubsystem.intakeSlideInternalPID(IntakeSubsystem.slideExtensionLimit);

                            if (delay(400))
                                outtakeSubsystem.turretSpinTo(OuttakeSubsystem.OuttakeTurretState.TRANSFER_BACK);
                            outtakeSubsystem.liftToInternalPID(0);

                            if (intakeTrajectory.isFinished() && delay(100))
                            {
//                                intakeSubsystem.intakeSlideMotorRawControl(-1);
                                intakedSpec = true;
                                internalTimerReset();
                            }
                        }
                        if (delay(140))
                        {
                            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER_BACK);
                            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.TRANSFER_BACK);
                            outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.DOWN);
                        }
                    }
                }
                else {
                    if (cycle != 2)
                    {
                        outtakeSubsystem.turretSpinTo(OuttakeSubsystem.OuttakeTurretState.TRANSFER_BACK);
                        if (hardware.drive.stopped())
                            intakeSubsystem.intakeSlideInternalPID(slideExtensionLimit, 0.6);
                    }
                }
                break;
            case DEPOSIT_DRIVE:
                Trajectory depositTrajectory = null;
                switch (cycle)
                {
                    case 2:
                        depositTrajectory = trajectories.firstDepositWhileTurning;
                        break;
                    case 3:
                        depositTrajectory = trajectories.secondDeposit;
                        break;
                    case 4:
                        depositTrajectory = trajectories.thirdDeposit;
                        break;
                    case 5:
                        depositTrajectory = trajectories.forthDeposit;
                        break;
                    case 6:
                        depositTrajectory = trajectories.fifthDeposit;
                        break;
                    case 7:
                        depositTrajectory = trajectories.sixthDeposit;
                        break;
                }
                if (depositTrajectory != null) {
                    hardware.drive.followTrajectorySplineHeading(depositTrajectory);

                    if (!transferred && cycle != 2) {
                        if (delay(500) && intakeSubsystem.isSlidesAtBase()) {
                            outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                            transferred = true;
                            internalTimerReset();
                        }
                        outtakeSubsystem.liftToInternalPID(0);
                        outtakeSubsystem.turretSpinTo(OuttakeSubsystem.OuttakeTurretState.TRANSFER_BACK);
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER_BACK);
                        outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.TRANSFER_BACK);
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.TRANSFER_BACK);
                        intakeSubsystem.intakeSlideMotorRawControl(-1);
                    } else {
                        if (cycle != 2) {
                            // ends the transfer
                            if (internalDelay(600)) {
                                intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.OFF);
                            } else if (internalDelay(90))
                                intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                        }
                        if (delay(40))
                        {
                            if (cycle != 2) // we need to flip when coming from the wall one
                                outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.RIGHT);
                            else outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.LEFT);
                        }
                        if (internalDelay(160)) {
                            intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                            if (internalDelay(300))
                                outtakeSubsystem.turretSpinTo(OuttakeSubsystem.OuttakeTurretState.SPEC_DEPOSIT_BACK);
                            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN_HIGH_AUTO_SCORE);
                            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SPECIMEN_HIGH_AUTO);
                            outtakeSubsystem.liftToInternalPID( cycle != 2 ? outtakeSubsystem.liftHighBarBackAutoPos + 1 : // slack on outtake makes so the arm is under...
                                    outtakeSubsystem.liftHighBarBackAutoPos + 2);

                            if (internalDelay(130)) {
                                intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HALF_DOWN);
                                intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);
                            }
                        }
                        if (internalDelay(500)) intakeSubsystem.intakeSlideInternalPID(4);
                        if (depositTrajectory.isFinished() && internalDelay(500) && delay(600))
                        {
                            state = autoState.DROP;
                            resetTimer();
                            break;
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
                    outtakeSubsystem.liftMotorRawControl(-1);
                    if (delay(100)) outtakeSubsystem.wristSetPos(0.75);
//                    if (delay(400)) intakeSubsystem.intakeSlideInternalPID(4);
                    if (delay(250)) outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.INTAKE);
                    if (delay(500))
                    {
                        state = autoState.INTAKE;
//                        outtakeSubsystem.liftMotorRawControl(0);
                        cycle++;
                        transferred = false;
                        intakedSpec = false;

                        resetTimer();
                        internalTimerReset();
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
                if (delay(200))
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HORIZONTAL);
                intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.RIGHT);

                outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN_AUTO_PRELOADS);
                outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SPECIMEN_AUTO_PRELOADS);
                outtakeSubsystem.liftToInternalPID(outtakeSubsystem.liftHighBarPrealodAutoPos + 1.5);
                outtakeSubsystem.turretKeepToAngleTicks(0, trajectories.preloadTrajectory.getFinalPose().getHeading());
                intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);

                LLResult result = cameraHardware.getLatestResult();
                if (result != null)
                    samplePose = cameraHardware.ro2GoatMath();

                if ((intakeSubsystem.isDistance(7.75) && delay(200)) || delay(2000))
                {
                    outtakeState = OuttakeState.DROP;
                    outtakeSubsystem.liftMotorRawControl(-1);
                    intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                    intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                    Pose driveTo = calculatePose(result);
                    hardware.drive.setRunMode(MecanumDrive.RunMode.P2P);
                    hardware.drive.setTargetPose(driveTo);
                    cameraHardware.captureSnapshot("Spec auto");
                    resetTimer();
                    break;
                }
                break;
            case DROP:
                if (delay(100)) outtakeSubsystem.wristSetPos(0.75);
                if (delay(250)) outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);
                colourValue = intakeSubsystem.getColourValue(); // we attempt intake in this case
                double extendoDistance = samplePose.getY() - 10; // arm offset
                if (!armDown) {
                    if (intakeSubsystem.slideOverPosition(extendoDistance - 2))
                    {
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DOWN);
                        armDown = true;
                        internalTimerReset();
                    }
                    intakeSubsystem.intakeSlideInternalPID(extendoDistance);
                }
                else if (internalDelay(250))
                    intakeSubsystem.intakeSlideInternalPID(extendoDistance + 10);

                outtakeSubsystem.turretKeepToAngleTicks(0, hardware.drive.getPoseEstimate().getHeading());

                if ((colourValue >= 850) || delay(1700))
                {
                    state = autoState.SUB_TO_HP;
                    outtakeState = OuttakeState.COLOUR_CHECK;
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HORIZONTAL);
                    armDown = false; // just to sure lol
                    cycle++;
                    resetTimer();
                    break;
                }
                break;
        }
    }

    private Pose calculatePose(LLResult result) {

        double tx = result.getTxNC();
        double finalHeading = Math.toRadians(90) - Math.toRadians(tx);
        Vector offSetVector = new Vector(-0, 6); //new Vector(-6.8, 4.2); // as center of rotation of the robot is not the camera
        // heading was < 90
        // x = -4 moved back
        // x = 4 moved front
        // y = -4 moved right
        // y = 4 moved left

        offSetVector = offSetVector.rotated(-finalHeading);
        Pose pose = hardware.drive.getPoseEstimate();
        return new Pose(
                pose.getX() + offSetVector.getX(),
                pose.getY() + offSetVector.getY(),
                finalHeading
        );
    }

    private void subToHpOuttakeState()
    {
        switch (outtakeState)
        {
            case COLOUR_CHECK:
                if (delay(80))
                {
                  if (intakeSubsystem.checkColour(IntakeSubsystem.IntakeFilter.NEUTRAL)) // neutral as we don't re attempt intake
                  {
                      outtakeState = OuttakeState.TRANSFER_START;
                  }
                  else outtakeState = OuttakeState.WRONG_COLOUR;
                  resetTimer();
                  break;
                }
                break;
            case WRONG_COLOUR:
                intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.HP_REVERSE);
                if (delay(150))
                {
                    outtakeState = OuttakeState.TRANSFER_START;
                    resetTimer();
                    break;
                }
                break;
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
                    outtakeSubsystem.liftToInternalPID(-2);
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
                if (delay(250)) intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);
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
                if (delay(100)) hardware.drive.followTrajectorySplineHeading(trajectories.hpToSubIntake);
                outtakeSubsystem.liftToInternalPID(outtakeSubsystem.liftHighBarPrealodAutoPos);
                LLResult result = null;
                if (delay(200))
                {
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HORIZONTAL);
                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN_AUTO_PRELOADS);
                    outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SPECIMEN_AUTO_PRELOADS);
                    outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.LEFT);
                    outtakeSubsystem.turretKeepToAngleTicks(0, hardware.drive.getPoseEstimate().getHeading());
                    if (delay(350)) intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.RIGHT);
                    if (delay(400))
                    {
                        result = cameraHardware.getLatestResult();
                        if (result != null) samplePose = cameraHardware.ro2GoatMath();
                    }
                }
                else intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);

                if (trajectories.hpToSubIntake.isFinished() || (intakeSubsystem.isDistance(7.75) && delay(900)) || delay(2000))
                {
                    outtakeState = OuttakeState.DROP;
                    outtakeSubsystem.liftMotorRawControl(-1);
                    intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                    Pose driveTo = calculatePose(result);
                    hardware.drive.setTargetPose(driveTo);
                    hardware.drive.setRunMode(MecanumDrive.RunMode.P2P);
                    resetTimer();
                    internalTimerReset(); // we need to use the internal timer as the trajectory is timed out
                    armDown = false;
                    break;
                }
                break;
            case DROP:
                if (delay(100)) outtakeSubsystem.wristSetPos(0.75);
                if (delay(300)) outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);
                colourValue = intakeSubsystem.getColourValue(); // we attempt intake in this case
                double extendoDistance = samplePose.getY() - 10; // arm offset
                if (!armDown) {
                    if (intakeSubsystem.slideOverPosition(extendoDistance - 2))
                    {
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DOWN);
                        armDown = true;
                        internalTimerReset();
                    }
                    intakeSubsystem.intakeSlideInternalPID(extendoDistance);
                }
                else if (internalDelay(250))
                    intakeSubsystem.intakeSlideInternalPID(extendoDistance + 10);

                outtakeSubsystem.turretKeepToAngleTicks(0, hardware.drive.getPoseEstimate().getHeading());

                if (colourValue > 900 || delay(2400))
                {
                    state = autoState.PRELOADS_DROP;
                    pickupState = PickupState.COLOUR_CHECK;
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HORIZONTAL);
                    intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                    cycle++;
                    resetTimer();
                    break;
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
                } else if (delay(170)) intakeSubsystem.intakeSlideMotorRawControl(-1);

                if (intakeSubsystem.getSlidePositionIn() < 12) intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                if (intakeSubsystem.getSlidePositionIn() < 1 && delay(300))
                {
                    state = pickupCycle == 3 ? autoState.INTAKE : autoState.PRELOADS_INTAKE;
                    pickupState = PickupState.SAMPLE_PICKUP;
                    if (pickupCycle != 3) {
                        extendIntakeSlides(pickupCycle);
                        intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DOWN);
                    }
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
            case COLOUR_CHECK:
                if (delay(80))
                {
                    if (intakeSubsystem.checkColour(IntakeSubsystem.IntakeFilter.NEUTRAL)) // neutral as we don't re attempt intake
                    {
                        pickupState = PickupState.TRANSFER_START;
                    }
                    else pickupState = PickupState.WRONG_COLOUR;
                    resetTimer();
                    break;
                }
                break;
//                throw new RuntimeException("Ling Ling cannot see");
            case WRONG_COLOUR:
                intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.HP_REVERSE);
                if (delay(150))
                {
                    pickupState = PickupState.TRANSFER_START;
                    resetTimer();
                    break;
                }
//                throw new RuntimeException("Ling Ling is yellow");
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
                if (delay(270))
                {
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.EXTENDO_DOWN);
                    if (pickupCycle != 3)
                        extendIntakeSlides(pickupCycle);
                    else intakeSubsystem.intakeSlideInternalPID(0);
                }
                else if (delay(100))
                {
                    intakeSubsystem.intakeSlideMotorRawControl(0);
                }
                if (delay(130))
                {
                    intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                }
                break;
            case SAMPLE_DROP:
                if (pickupCycle != 3)
                {
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.EXTENDO_DOWN);
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
                intakeSubsystem.intakeSlideInternalPID(30);
                break;
            case 2:
                intakeSubsystem.intakeSlideInternalPID(28.3);
                break;
            case 3:
                intakeSubsystem.intakeSlideInternalPID(29.5);
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
