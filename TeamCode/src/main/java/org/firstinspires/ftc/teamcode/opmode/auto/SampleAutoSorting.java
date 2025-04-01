package org.firstinspires.ftc.teamcode.opmode.auto;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.gvf.MecanumDrive;
import org.firstinspires.ftc.teamcode.gvf.trajectories.Trajectory;
import org.firstinspires.ftc.teamcode.gvf.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.firstinspires.ftc.teamcode.gvf.utils.Vector;
import org.firstinspires.ftc.teamcode.opmode.auto.paths.SampleAutoPathNew;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;
import org.firstinspires.ftc.teamcode.system.vision.CameraHardware;

import java.util.ArrayList;

@Autonomous(name = "0+9 Sorting", group = "Close")
public class SampleAutoSorting extends LinearOpMode
{
     double pureYdis;
     boolean stopped;
     boolean exitSubIntake;

    enum autoState {
        PRELOAD_DEPOSIT,
        INTAKE,
        INTAKE_SUB,
        INTAKE_EDGE_CASE,
        DEPOSIT,
        PARK,
        IDLE
    }
    enum OuttakeState
    {
        TRANSFER_START,
        TRANSFER_END,
        READY,
        DROP
    }
    enum SubIntakeState
    {
        IDLE,
        CAMERA_DETECTION,
        ALIGN,
        INTAKE,
        MISSED,
        WRONG_COLOUR,
        SECOND_ATTEMPT,
        TINKO_BF_INTAKE
    }

    ElapsedTime GlobalTimer;
    autoState state = autoState.PRELOAD_DEPOSIT;
    OuttakeState outtakeState = OuttakeState.READY;
    SubIntakeState subIntakeState = SubIntakeState.IDLE;
    GeneralHardware hardware;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    SampleAutoPathNew trajectories = new SampleAutoPathNew();
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    CameraHardware cameraHardware;
//    InverseKinematics cameraInverseKinematics = new InverseKinematics();
    double globalTimer, sequenceTimer, internalTimer, intakeClipTimer, turretTimer;
    int cycle = 0;
    boolean intaked = false;
    double yPosition, xPosition, heading, headingAngleDeg;
    double sampleThreshold;
    double headingErrorToEndPose;
    boolean dropped = false;
    double intakeSubTarget = 8;
    double slideCachedTarget;
    boolean cachedSlideTarget, cachedPoseTarget;

    Pose intakePose = null;
    LLResult result = null;
    Vector sampleDis;
    Trajectory depositTrajectory;
    Trajectory reverseTrajectory = null;

    @Override
    public void runOpMode() throws InterruptedException
    {
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.RED, true);
        hardware.drive.setRunMode(MecanumDrive.RunMode.Vector);
        hardware.drive.getLocalizer().setOffSet(trajectories.closeStartPose);
        intakeSubsystem = new IntakeSubsystem(hardware);
        outtakeSubsystem = new OuttakeSubsystem(hardware);
        cameraHardware = new CameraHardware(hardware);
        cameraHardware.pipelineSwitch(CameraHardware.PipelineType.DETECTOR_YELLOW);
        GlobalTimer = new ElapsedTime(System.nanoTime());
        globalTimer = GlobalTimer.milliseconds();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        resetTimer();

        while (!isStarted())
        {
            outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.INTAKE);
            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SAMPLE);
            outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.RIGHT);

            intakeSubsystem.armSetPos(0.65);
            intakeSubsystem.intakeTurretSetPos(0.65);


            globalTimer = GlobalTimer.milliseconds();
        }
        waitForStart();
        globalTimer = GlobalTimer.milliseconds();
        resetTimer();
        hardware.resetCacheHubs();
        while (opModeIsActive())
        {
            globalTimer = GlobalTimer.milliseconds();
            intakeSubsystem.intakeReads(state == autoState.INTAKE || state == autoState.INTAKE_SUB);
            outtakeSubsystem.outtakeReads();

            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            autoSequence();
            hardware.update();
            Pose poseEstimate = hardware.drive.getPoseEstimate();
            yPosition = poseEstimate.getY();
            xPosition = poseEstimate.getX();
            heading = poseEstimate.getHeading();
            headingAngleDeg = Math.toDegrees(heading);
            DashboardUtil.drawRobot(fieldOverlay, poseEstimate.toPose2d(), true, "red");
            DashboardUtil.drawRobot(fieldOverlay, hardware.drive.getPredictedPoseEstimate().toPose2d(), true);
            DashboardUtil.drawCurve(fieldOverlay, hardware.drive.trajectoryFollowing);
            if (intakePose != null)
                DashboardUtil.drawRobot(fieldOverlay, intakePose.toPose2d(), true, "orange");
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("State", state);
            telemetry.addData("Outtake state", outtakeState);
            telemetry.addData("Sub intake state", subIntakeState);
            telemetry.addData("Sample dis", sampleDis);
            telemetry.addData("Pure y dis", pureYdis);
            telemetry.addData("Cycle", cycle);
            telemetry.addLine();
            telemetry.addData("Pose", hardware.drive.getPoseEstimate());
            telemetry.addData("Target pose sample", intakePose);
            telemetry.addLine();
            telemetry.addData("Camera Result", result != null);
            telemetry.addData("Valid Result?", result != null ? result.isValid() : null); // doesn't raise
            telemetry.update();
        }
    }
    public void autoSequence()
    {
        switch (state)
        {
            case PRELOAD_DEPOSIT:
                hardware.drive.followTrajectorySplineHeading(trajectories.preloadTrajectory);
                outtakeKeepTurretBack();
                switch (outtakeState)
                {
                    case READY:
                        if (trajectories.preloadTrajectory.isFinished() && delay(300))
                        {
                            outtakeState = OuttakeState.DROP;
                            resetTimer();
                            break;
                        }
                        if (delay(40))
                        {
                            liftToMaxHeight();
                            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SAMPLE);
                            outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.RIGHT);
                            if (delay(400))
                            {
                                intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DOWN);
                                intakeSubsystem.intakeSlideInternalPID(22);
                            }
                            if (delay(410)) outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SAMPLE); // arm go later so we don't like hit the bucket
                            if (delay(600)) intakeSubsystem.intakeTurretSetPos(0.58);
                            else if (delay(200)) intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);
                        }
                        break;
                    case DROP:
                        outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                        if (delay(140))
                        {
                            state = autoState.INTAKE;
                            cycle += 2;
                            intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                            resetTimer();
                            break;
                        }
                        break;
                }
                break;
            case INTAKE:
                Trajectory intakeTrajectory = null;
                double slideTarget;
                switch (cycle)
                {
                    case 2:
                        intakeTrajectory = trajectories.firstIntake;
                        break;
                    case 3:
                        intakeTrajectory = trajectories.secondIntake;
                        break;
                    case 4:
                        intakeTrajectory = trajectories.thirdIntake;
                        break;
                }
                if (intakeTrajectory != null)
                {
                    if ((cycle != 2 && delay(100)) || delay(150))
                        hardware.drive.followTrajectorySplineHeading(intakeTrajectory);
                    intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                    if (delay(40))
                    {
                        intakeSubsystem.intakeSlideInternalPID(21);
                        intakeSubsystem.intakeTurretSetPos(0.58);
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DOWN);
                    }
                    if (delay(120)) // sets up for the transfer
                    {
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER_BACK);
                        outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.TRANSFER_BACK);
                        outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.DOWN);
                        if (delay(200)) liftToBase(-0.5);
                        if (delay(300))
                        {
                            outtakeKeepTurretBack();
                        }
                    }
                    if (delay(2500) || intakeSubsystem.getColorValue() > 950)
                    {
                        state =  autoState.DEPOSIT;
                        outtakeState = OuttakeState.TRANSFER_START;
                        resetTimer();
                        break;
                    }

                }
                break;
            case DEPOSIT:
                depositTrajectory = null;
                switch (cycle)
                {
                    case 1:
                        depositTrajectory = trajectories.hpDeposit;
                        break;
                    case 2:
                        depositTrajectory = trajectories.firstDeposit;
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
                    case 8:
                        depositTrajectory = trajectories.seventhDeposit;
                        break;
                    case 9:
                        depositTrajectory = trajectories.eighthDeposit;
                        break;
                }
                if (depositTrajectory != null)
                {
                    hardware.drive.followTrajectorySplineHeading(depositTrajectory);
                }
                switch (outtakeState)
                {
                    case TRANSFER_START:
                        if (delay(20))
                        {
                            intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);
                            intakeSubsystem.intakeSlideMotorRawControl(-1);
                            outtakeKeepTurretBack();
                            liftToBase(-0.5);
                            if (delay(50))
                            {
                                intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.TRANSFER_BACK);
                            }
                            if (intakeSubsystem.isSlidesAtBase() && delay(140))
                            {
                                outtakeState = OuttakeState.TRANSFER_END;
                                resetTimer();
                                break;
                            }
                        }
                        break;
                    case TRANSFER_END:
                        if (delay(40))
                        {
                            outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                            if (delay(180))
                            {
                                liftToMaxHeight();
                                if (delay(300))
                                {
                                    outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SAMPLE);
                                    outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.DOWN);
                                    outtakeSubsystem.turretKeepToAngleTicks(135, headingAngleDeg);
                                    intakeSubsystem.intakeSpin(0);
                                    if (cycle > 4 || delay(500)) outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SAMPLE); // lift has enougth time to be high
                                    if(cycle < 4)
                                    {
                                        intakeSubsystem.intakeSlideInternalPID(21);
                                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DOWN);
                                        if (delay(500)) intakeSubsystem.intakeTurretSetPos(0.58);
                                    }
                                }
                                else if (delay(130))
                                {
                                    intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                                }
                                if (delay(650) && depositTrajectory.isFinished() && outtakeSubsystem.ticksToInchesSlidesMotor(outtakeSubsystem.liftPosition) > 29.5)
                                {
                                    outtakeState = OuttakeState.DROP;
                                    resetTimer();
                                    break;
                                }
                            } else outtakeSubsystem.liftMotorRawControl(0);
                        }
                        break;
                    case DROP:
                        if (delay(90))
                        {
                            state = cycle >= 4 ? autoState.INTAKE_SUB : autoState.INTAKE;
                            outtakeSubsystem.liftMotorRawControl(0);
                            if (cycle < 4)
                            {
                                intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                            }
                            else intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HORIZONTAL);
                            subIntakeState = SubIntakeState.IDLE;
                            exitSubIntake = false;
                            cycle++;
                            resetTimer();
                            break;
                        }
                        outtakeSubsystem.turretKeepToAngleTicks(135, headingAngleDeg);
                        outtakeSubsystem.liftMotorRawControl(0);
                        outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                        break;
                }
                break;
            case INTAKE_SUB:
                Trajectory subIntakeTrajectory = null;
                switch (cycle)
                {
                    case 5:
                        subIntakeTrajectory = trajectories.submersibleIntake;
                        break;
                    case 6:
                        subIntakeTrajectory = trajectories.submersibleIntakeSecond;
                        break;
                    case 7:
                        subIntakeTrajectory = trajectories.submersibleIntakeThird;
                        break;
                    case 8:
                        subIntakeTrajectory = trajectories.submersibleIntakeForth;
                        break;
                    case 9:
                        subIntakeTrajectory = trajectories.submersibleIntakeFifth;
                        break;
                }
                if (subIntakeTrajectory != null)
                {
                    if (subIntakeState == SubIntakeState.IDLE) hardware.drive.followTrajectorySplineHeading(subIntakeTrajectory);
                    if(delay(300))
                    {
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER_BACK);
                        outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.TRANSFER_BACK);
                        outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.DOWN);
                        cameraHardware.start();
                        if (!stopped)
                        {
                            stopped = trajectories.submersibleIntake.isFinished() && hardware.drive.stopped();
                            if (stopped)
                            {
                                cameraHardware.captureSnapshot("Auto");
                            }
                            internalTimerReset();
                        }
                        if (stopped)
                        {
                            switch (subIntakeState)
                            {
                                case IDLE:
                                    if (internalDelay(20)) {
                                        subIntakeState = SubIntakeState.CAMERA_DETECTION;
                                        internalTimerReset();
                                    }
                                    break;
                                case CAMERA_DETECTION:
                                    result = cameraHardware.getLatestResult();
                                    if (result == null || !result.isValid() || result.getDetectorResults().isEmpty())
                                    {
                                        subIntakeState = SubIntakeState.TINKO_BF_INTAKE;
                                        internalTimerReset();
                                        break;
                                    }
                                    ArrayList<CameraHardware.Sample> samples = cameraHardware.sampleQuery(result, hardware.drive.getPoseEstimate());
                                    CameraHardware.Sample intakeSample = samples.get(0); // grab the closest to the crosshair
                                    for (CameraHardware.Sample sample : samples)
                                    {
                                        if (sample.angle < 60)
                                        {
                                            intakeSample = sample;
                                            break;
                                        }
                                    }
                                    sampleDis = new Vector(intakeSample.vectorToDetectPose.getX(), intakeSample.vectorToDetectPose.getY()); // InverseKinematics.distanceToSample(result.getTyNC(), result.getTxNC());
                                    sampleDis.setY(sampleDis.getY() - 10);
                                    double finalHeading = Math.toRadians(0) - Math.toRadians(intakeSample.detectorResult.getTargetXDegreesNoCrosshair());
                                    Vector offSetVector = new Vector(4, 6.5); // as center of rotation of the robot is not the camera
                                    offSetVector = offSetVector.rotated(-finalHeading);
                                    intakePose = new Pose(
                                            subIntakeTrajectory.getFinalPose().getX() + offSetVector.getX(),
                                            subIntakeTrajectory.getFinalPose().getY() + offSetVector.getY(),
                                            finalHeading);

                                    double removeDisFromSlides = intakePose.getX() - subIntakeTrajectory.getFinalPose().getX();
                                    sampleDis.setY(sampleDis.getY() - removeDisFromSlides);
//                                    if (sampleDis.getY() < 6) sampleDis.setY(sampleDis.getY() - 1); // = new Vector(sampleDis.getX(), sampleDis.getY() - 1); // the detection is a little bit iffy when is close
                                    if (sampleDis.getY() < 0) sampleDis.setY(1); // = new Vector(sampleDis.getX(), 1);
                                    subIntakeState = SubIntakeState.ALIGN;
                                    internalTimerReset();
                                    break;
                                case ALIGN: // align always runs now
                                    hardware.drive.setRunMode(MecanumDrive.RunMode.P2P);
                                    hardware.drive.setTargetPose(intakePose);

                                    intakeSubsystem.intakeSlideInternalPID(sampleDis.getY());
                                    if (internalDelay(300))
                                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HALF_DOWN);
                                    if (((hardware.drive.reachedTargetAndHeading(1) && intakeSubsystem.slideOverPosition(sampleDis.getY() * 0.9))
                                            || internalDelay(700)))
                                    {
                                        subIntakeState = SubIntakeState.INTAKE;
                                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DOWN);
                                        internalTimerReset();
                                    }
                                    break;
                                case INTAKE:
                                    if (intakeSubsystem.getColourSensorDistance() < 0.3)
                                    {
                                        if (intakeSubsystem.checkColour(IntakeSubsystem.IntakeFilter.NEUTRAL) || true)
                                        {
                                            exitSubIntake = true;
                                            break;
                                        }
                                        else
                                        {
                                            subIntakeState = SubIntakeState.WRONG_COLOUR;
                                            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HALF_DOWN);
                                            internalTimerReset();
                                        }
                                    }
                                    if (internalDelay(90)) intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                                    if (internalDelay(400)) intakeSubsystem.intakeSlideInternalPID(sampleDis.getY() + 5);
                                    if (internalDelay(850)) subIntakeState = SubIntakeState.MISSED;

                                    break;
                                case MISSED:
                                    intakeSubsystem.intakeSlideInternalPID(sampleDis.getY() + 10);
                                    if (internalDelay(1000))
                                    {
                                        subIntakeState = SubIntakeState.SECOND_ATTEMPT;
                                        internalTimerReset();
                                    }
                                    break;
                                case WRONG_COLOUR:
                                    if (internalDelay(100)) intakeSubsystem.intakeSpin(-0.5);
                                    intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.MAX_LEFT);
                                    if (intakeSubsystem.getColourSensorDistance() > 0.45 || internalDelay(250)) exitSubIntake = true;
                                    if  (internalDelay(400))
                                        throw new RuntimeException("Wrong colour");
                                    break;
                                case SECOND_ATTEMPT:
                                    intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                                    if (internalDelay(700)) intakeSubsystem.intakeSlideInternalPID(sampleDis.getY() + 10);
                                    else if (internalDelay(300))
                                    {
                                        hardware.drive.setTargetPose( new Pose(intakePose.getX(), intakePose.getY(),
                                                intakePose.getHeading() + intakePose.getHeading() > 0 ? Math.toRadians(5) : -Math.toRadians(5)));
                                    }
                                    else
                                    {
                                        intakeSubsystem.intakeSlideInternalPID(sampleDis.getY() + 5);
                                        if (heading > 0) intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.LEFT);
                                        else intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.RIGHT);
                                    }
                                    if (internalDelay(1200))
                                    {
                                        if (intakeSubsystem.checkColour(IntakeSubsystem.IntakeFilter.NEUTRAL) || true)
                                        {
                                            exitSubIntake = true;
                                            break;
                                        }
                                        else
                                        {
                                            subIntakeState = SubIntakeState.WRONG_COLOUR;
                                            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HALF_DOWN);
                                            internalTimerReset();
                                        }
                                    }
                                    break;
                                case TINKO_BF_INTAKE: // if we don't see anything, we attempt blind intake
                                    sampleDis = new Vector(0, 14);
                                    intakePose = new Pose(subIntakeTrajectory.getFinalPose());
                                    subIntakeState = SubIntakeState.ALIGN;
                                    internalTimerReset();
//                                    throw new RuntimeException("BLIND MotherFucker");
                                    break;
                            }
                        }
                        liftToBase();
                        outtakeKeepTurretBack();
                    }
                    if (exitSubIntake)
                    {
                        state = autoState.DEPOSIT;
                        outtakeState = OuttakeState.TRANSFER_START;
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.TRANSFER_BACK);
                        stopped = false;
//                        if (!intakeSubsystem.checkColour(IntakeSubsystem.IntakeFilter.NEUTRAL)) throw new RuntimeException("Wrong colour");
//                        sampleDis = null;
//                        intakePose = null;
//                        result = null;
                        resetTimer();
                        break;
                    }
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
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                    outtakeSubsystem.liftToInternalPID(4);
                }
                break;
            case IDLE: // we idle here duuhhh
                outtakeSubsystem.liftMotorRawControl(0);
                intakeSubsystem.intakeSlideMotorRawControl(0);
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
    public void outtakeKeepTurretBack()
    {
        if (outtakeSubsystem.turretReached(180))
        {
            if (globalTimer - turretTimer > 100) // we can save battery if this motor is off
            {
                outtakeSubsystem.turretRawControl(0);
            } else
            {
                outtakeSubsystem.turretSpinToGains(OuttakeSubsystem.OuttakeTurretState.TRANSFER_BACK);
            }
        } else
        {
            outtakeSubsystem.turretSpinToGains(OuttakeSubsystem.OuttakeTurretState.TRANSFER_BACK);
            turretTimer = globalTimer;
        }
    }
    public void liftToMaxHeight()
    {
        if (outtakeSubsystem.ticksToInchesSlidesMotor(outtakeSubsystem.liftPosition) > 31)
            outtakeSubsystem.liftToInternalPID(32.45); // max heigh 26.1
        else outtakeSubsystem.liftMotorRawControl(1);
    }
    public void liftToBase()
    {
        liftToBase(0);
    }
    public void liftToBase(double inOffset)
    {
        if (outtakeSubsystem.ticksToInchesSlidesMotor(outtakeSubsystem.liftPosition) < 1)
            outtakeSubsystem.liftToInternalPID(0 + inOffset);
        else outtakeSubsystem.liftMotorRawControl(-1);
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
