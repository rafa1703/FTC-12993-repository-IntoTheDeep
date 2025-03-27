package org.firstinspires.ftc.teamcode.opmode.auto;


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
import org.firstinspires.ftc.teamcode.opmode.auto.paths.SampleAutoPathNew;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;
import org.firstinspires.ftc.teamcode.system.vision.CameraHardware;
import org.firstinspires.ftc.teamcode.system.vision.InverseKinematics;

@Autonomous(name = "0+7 turning intake", group = "Close")
public class SampleAutoGoated extends LinearOpMode
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
    SampleAutoPathNew trajectories = new SampleAutoPathNew();
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    CameraHardware cameraHardware;
    InverseKinematics cameraInverseKinematics = new InverseKinematics();
    double globalTimer, sequenceTimer, intakeClipTimer, turretTimer;
    int cycle = 0;
    boolean intaked = false;
    double yPosition, xPosition, headingPosition, headingAngleDeg;
    double sampleThreshold;
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
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.RED, true);
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
            headingPosition = poseEstimate.getHeading();
            headingAngleDeg = Math.toDegrees(headingPosition);
            DashboardUtil.drawRobot(fieldOverlay, poseEstimate.toPose2d(), true, "red");
            DashboardUtil.drawRobot(fieldOverlay, hardware.drive.getPredictedPoseEstimate().toPose2d(), true);
            DashboardUtil.drawCurve(fieldOverlay, hardware.drive.trajectoryFollowing);
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("State", state);
            telemetry.addData("Outtake state", outtakeState);
            telemetry.addData("Cycle", cycle);
            telemetry.addLine();
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
                            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SAMPLE);
                            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SAMPLE);
                            outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.RIGHT);
                            if (delay(400))
                            {
                                intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DOWN);
                                intakeSubsystem.intakeSlideInternalPID(22);
                            }
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
                    if (delay(100))
                        hardware.drive.followTrajectorySplineHeading(intakeTrajectory);
                    intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                    if (delay(40))
                    {
                        outtakeKeepTurretBack();
                        intakeSubsystem.intakeSlideInternalPID(21);
                        intakeSubsystem.intakeTurretSetPos(0.58);
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DOWN);

                    }
                    if (delay(120)) // sets up for the transfer
                    {
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER_BACK);
                        outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.TRANSFER_BACK);
                        outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.DOWN);
                        if (delay(300)) liftToBase();
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
                                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SAMPLE);
                                    outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.DOWN);
                                    outtakeSubsystem.turretKeepToAngleTicks(135, headingAngleDeg);
                                    intakeSubsystem.intakeSpin(0);

                                    intakeSubsystem.intakeSlideInternalPID(21);
                                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DOWN);
                                    if (delay(500)) intakeSubsystem.intakeTurretSetPos(0.58);
                                }
                                else if (delay(130))
                                {
                                    intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
                                }
                                if (delay(500) && depositTrajectory.isFinished() && outtakeSubsystem.ticksToInchesSlidesMotor(outtakeSubsystem.liftPosition) > 24)
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
                }
                if (subIntakeTrajectory != null)
                {

//                    if (subIntakeTrajectory.isFinished())
//                    {
//                        if (cameraHardware.getLatestResult().isValid() && !cachedPoseTarget)
//                        {
//                            Pose allingPose = preloadPose.plus(new Pose(0, 0, Math.toRadians(cameraHardware.getLatestResult().getTx())));
//                            hardware.drive.setTargetPose(allingPose);
//                            cachedPoseTarget = true;
//                        }
//                        else hardware.drive.setTargetPose(preloadPose);
//                    }
//                    else hardware.drive.followTrajectorySplineHeading(subIntakeTrajectory);
//
//                    if (hardware.drive.reachedHeading(Math.toRadians(2)))
//                    {
//                        if (!cachedSlideTarget)
//                        {
//                            slideCachedTarget = cameraInverseKinematics.distanceToSample(cameraHardware.getLatestResult().getTy()); // cache distance because the slides will get in front of the camera
//                            slideCachedTarget -= 6.5;
//                            cachedSlideTarget = true;
//                        }
//                        else if (delay(90)) intakeSubsystem.intakeSlideInternalPID(slideCachedTarget);
//                    }
                    hardware.drive.followTrajectorySplineHeading(subIntakeTrajectory);
                    if(delay(300))
                    {
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER_BACK);
                        outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.TRANSFER_BACK);
                        outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.DOWN);

                        if ( cycle == 5 ? xPosition > 39 : xPosition > -24) // TODO: no vision
                        {
                            if (intakeSubsystem.slideOverPosition(12))
                            {
                                intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DOWN);
                                intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                                intakeSubsystem.intakeSlideInternalPID(19);
                            } else intakeSubsystem.intakeSlideInternalPID(14);

                        }
                        liftToBase();
                        outtakeKeepTurretBack();
                    }
                    if ((intakeSubsystem.getColorValue() > 1100 && delay(150)) || delay(3000))
                    {
                        state = autoState.DEPOSIT;
                        outtakeState = OuttakeState.TRANSFER_START;
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.TRANSFER_BACK);
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
        if (outtakeSubsystem.ticksToInchesSlidesMotor(outtakeSubsystem.liftPosition) > 25)
            outtakeSubsystem.liftToInternalPID(26.1);
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
}
