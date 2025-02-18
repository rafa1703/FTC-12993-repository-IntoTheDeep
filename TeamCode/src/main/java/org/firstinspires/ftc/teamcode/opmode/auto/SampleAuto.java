package org.firstinspires.ftc.teamcode.opmode.auto;


import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideExtensionLimit;
import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTeleBase;
import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTransfer;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.gvf.MecanumDrive;
import org.firstinspires.ftc.teamcode.gvf.trajectories.Trajectory;
import org.firstinspires.ftc.teamcode.gvf.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.firstinspires.ftc.teamcode.opmode.auto.paths.SampleAutoPath;
import org.firstinspires.ftc.teamcode.system.accessory.math.Angles;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;
import org.firstinspires.ftc.teamcode.system.vision.CameraHardware;
import org.firstinspires.ftc.teamcode.system.vision.InverseKinematics;

@Autonomous(name = "0+7", group = "Close")
public class SampleAuto extends LinearOpMode
{
    enum autoState {
        PRELOAD_DEPOSIT,
        HP_INTAKE,
        HP_DEPOSIT,
        INTAKE,
        INTAKE_SUB,
        INTAKE_DROP,
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

    ElapsedTime GlobalTimer;
    autoState state = autoState.PRELOAD_DEPOSIT;
    OuttakeState outtakeState = OuttakeState.READY;
    GeneralHardware hardware;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    SampleAutoPath trajectories = new SampleAutoPath();
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    CameraHardware cameraHardware;
    InverseKinematics cameraInverseKinematics = new InverseKinematics();
    double globalTimer, sequenceTimer, intakeClipTimer, turretTimer;
    int cycle = 0;
    boolean intaked = false;
    double yPosition, xPosition, headingPosition, headingAngleDeg;
    double headingErrorToEndPose;
    boolean dropped = false;
    double intakeSubTarget = 8;
    double slideCachedTarget;
    boolean cachedSlideTarget, cachedPoseTarget;
    Pose preloadPose = trajectories.submersibleIntake.getFinalPose();
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
        cameraHardware = new CameraHardware(hardware);
        GlobalTimer = new ElapsedTime(System.nanoTime());
        globalTimer = GlobalTimer.milliseconds();
        resetTimer();

        while (!isStarted())
        {
            intakeSubsystem.intakeFilter = IntakeSubsystem.IntakeFilter.NEUTRAL;
            outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
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
            intakeSubsystem.intakeReads(state == autoState.INTAKE || state == autoState.INTAKE_SUB);
            outtakeSubsystem.outtakeReads();

            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            autoSequence();
            hardware.update();
            Pose poseEstimate = hardware.drive.getPoseEstimate();
            yPosition = poseEstimate.getY();
            xPosition = poseEstimate.getX();
            headingPosition = poseEstimate.getHeading();
            headingAngleDeg = Math.toDegrees(headingPosition);
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
                hardware.drive.followTrajectorySplineHeading(trajectories.preloadTrajectory);
                switch (outtakeState)
                {
                    case READY:
                        if (trajectories.preloadTrajectory.isFinished() && delay(150))
                        {
                            outtakeState = OuttakeState.DROP;
                            resetTimer();
                            break;
                        }
                        if (delay(40))
                        {
                            outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftHighBucketPos);
                            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SAMPLE);
                            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SAMPLE);
                            outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.UP);
                        }
                        if (delay(300))
                        {
                            outtakeSubsystem.turretSpinTo(225);
                        }
                        break;
                    case DROP:
                        outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                        if (delay(140))
                        {
                            state = autoState.HP_INTAKE;
                            intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
                            cycle++;
                            resetTimer();
                            break;
                        }
                        break;
                }
                break;
            case HP_INTAKE:
                hardware.drive.followTrajectorySplineHeading(trajectories.hpIntake);
                if (delay(90))
                {
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DOWN);
                    intakeSubsystem.intakeSlideInternalPID(slideExtensionLimit);
                    intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                    if (delay(120))
                    {
                        outtakeSubsystem.liftToInternalPID(0);
                        turretSpinTo(180, OuttakeSubsystem.OuttakeArmServoState.TRANSFER_BACK,
                                OuttakeSubsystem.OuttakeWristServoState.TRANSFER_BACK);
                    }
                    if (intakeSubsystem.getColorValue() > 900 && delay(240))
                    {
                        state = autoState.HP_DEPOSIT;
                        resetTimer();
                        break;
                    }
                }
                break;
            case HP_DEPOSIT:
                hardware.drive.followTrajectory(trajectories.hpDeposit);
                switch (outtakeState)
                {
                    case TRANSFER_START:
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.TRANSFER_BACK);
                        if (delay(20)) intakeSubsystem.intakeSlideInternalPID(-2);
                        if (intakeSubsystem.isSlidesAtBase() && delay(50))
                        {
                            outtakeState = OuttakeState.TRANSFER_END;
                            resetTimer();
                            break;
                        }
                        break;
                    case TRANSFER_END:
                        if (delay(40))
                        {
                            outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                            if (delay(180))
                            {
                                outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftHighBucketPos);
                                if (delay(230))
                                {
                                    outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SAMPLE);
                                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SAMPLE);
                                    outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.UP);
                                    outtakeSubsystem.turretSpinTo(225);

                                    intakeSubsystem.intakeSlideInternalPID(slideExtensionLimit);
                                    intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.MAX_LEFT);
                                    intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.OFF);
                                }
                                else if (delay(130))
                                {
                                    intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                                }
                                if (delay(300) && trajectories.hpDeposit.isFinished() &&
                                        outtakeSubsystem.turretReached(255) &&
                                        outtakeSubsystem.liftReached(OuttakeSubsystem.liftHighBucketPos))
                                {
                                    outtakeState = OuttakeState.DROP;
                                    resetTimer();
                                    break;
                                }
                            }
                        }
                        break;
                    case DROP:
                        if (delay(90))
                        {
                            state = autoState.INTAKE;
                            cycle++;
                            resetTimer();
                            break;
                        }
                        outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                        break;

                }
                break;
            case INTAKE:
                Trajectory intakeTrajectory = null;
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
                    hardware.drive.followTrajectorySplineHeading(intakeTrajectory);
                    if (delay(40))
                    {
                        intakeSubsystem.intakeSlideInternalPID(slideExtensionLimit);
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                    }
                    if (delay(120)) // sets up for the transfer
                    {
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER_BACK);
                        outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftBasePos);
                        outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.TRANSFER_BACK);
                        outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.UP);
                    }
                    if (intakeSubsystem.getColorValue() > 500 && delay(190))
                    {
                        state =  autoState.DEPOSIT;
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
                        depositTrajectory = trajectories.firstDeposit;
                        break;
                    case 2:
                        depositTrajectory = trajectories.secondDeposit;
                        break;
                    case 3:
                        depositTrajectory = trajectories.thirdDeposit;
                        break;
                    case 4:
                        depositTrajectory = trajectories.forthDeposit;
                        break;
                    case 5:
                        depositTrajectory = trajectories.fifthDeposit;
                        break;
                }
                if (depositTrajectory != null) hardware.drive.followTrajectorySplineHeading(depositTrajectory);
                switch (outtakeState)
                {
                    case TRANSFER_START:
                        if (delay(20))
                        {
                            intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);
                            intakeSubsystem.intakeSlideInternalPID(-2);
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
                                outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftHighBucketPos);
                                if (delay(230))
                                {
                                    outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SAMPLE);
                                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SAMPLE);
                                    outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.UP);
                                    outtakeSubsystem.turretKeepToAngle(-135, headingAngleDeg);
                                    if (cycle != 4)
                                    {
                                        intakeSubsystem.intakeSlideInternalPID(slideExtensionLimit);
                                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DOWN);
                                        intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.MAX_LEFT);
                                    }
                                    else
                                    {
                                        intakeSubsystem.intakeSlideInternalPID(slideTeleBase);
                                        intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);
                                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.IN);
                                    }
                                    intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.OFF);
                                }
                                else if (delay(130))
                                {
                                    intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                                }
                                if (delay(300) && depositTrajectory.isFinished() &&
                                        outtakeSubsystem.turretReached() &&
                                        outtakeSubsystem.liftReached(OuttakeSubsystem.liftHighBucketPos))
                                {
                                    outtakeState = OuttakeState.DROP;
                                    resetTimer();
                                    break;
                                }
                            }
                        }
                        break;
                    case DROP:
                        if (delay(90))
                        {
                            state = cycle > 4 ? autoState.INTAKE_SUB : autoState.INTAKE;
                            cycle++;
                            resetTimer();
                            break;
                        }
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
                }
                if (subIntakeTrajectory != null)
                {

                    if (subIntakeTrajectory.isFinished())
                    {
                        if (cameraHardware.getLatestResult().isValid() && !cachedPoseTarget)
                        {
                            Pose allingPose = preloadPose.plus(new Pose(0, 0, Math.toRadians(cameraHardware.getLatestResult().getTx())));
                            hardware.drive.setTargetPose(allingPose);
                            cachedPoseTarget = true;
                        }
                        else hardware.drive.setTargetPose(preloadPose);
                    }
                    else hardware.drive.followTrajectorySplineHeading(subIntakeTrajectory);

                    if (hardware.drive.reachedHeading(Math.toRadians(2)))
                    {
                        if (!cachedSlideTarget)
                        {
                            slideCachedTarget = cameraInverseKinematics.distanceToSample(cameraHardware.getLatestResult().getTy()); // cache distance because the slides will get in front of the camera
                            slideCachedTarget -= 6.5;
                            cachedSlideTarget = true;
                        }
                        else if (delay(90)) intakeSubsystem.intakeSlideInternalPID(slideCachedTarget);
                    }
                    if(delay(100))
                    {
                        outtakeSubsystem.liftTo(0);
                        turretSpinTo(0, OuttakeSubsystem.OuttakeArmServoState.TRANSFER_FRONT,
                                OuttakeSubsystem.OuttakeWristServoState.TRANSFER_FRONT);
                    }
                    if ((intakeSubsystem.getColorValue() > 800 && delay(150)) || delay(1200))
                    {
                        state = autoState.DEPOSIT;
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HALF_TRANSFER);
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
                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.HALF_TRANSFER);
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                    outtakeSubsystem.liftToInternalPIDTicks(100);
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
                outtakeSubsystem.turretSpinTo(targetAngle);
            }
        }
        else
        {
            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPIN);
            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SPIN);
            outtakeSubsystem.turretSpinTo(targetAngle); // we
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
}
