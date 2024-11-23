/*

package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTeleBase;
import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTeleTransfer;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.gvf.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

public class redCloseAuto extends LinearOpMode
{

    enum autoState {
        INTAKE,
        TRANSFER_START,
        TRANSFER_END,
        DEPOSIT_DRIVE,
        DROP,
        RETURN,
        PARK,
        IDLE
    }
    ElapsedTime GlobalTimer;
    autoState state = autoState.DEPOSIT_DRIVE;
    GeneralHardware hardware;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Paths trajectories = new Paths();
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    double globalTimer, sequenceTimer, intakeClipTimer;
    int cycle = 0;
    boolean dropped = false;

    @Override
    public void runOpMode() throws InterruptedException
    {
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.Red, true);
        hardware.startThreads(this);
        intakeSubsystem = new IntakeSubsystem(hardware);
        outtakeSubsystem = new OuttakeSubsystem(hardware);
        GlobalTimer = new ElapsedTime(System.nanoTime());
        globalTimer = GlobalTimer.milliseconds();
        resetTimer();

        while (!isStarted())
        {
            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.LOW);
            intakeSubsystem.intakeChute(IntakeSubsystem.IntakeChuteServoState.UP);
            intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.HOLD);
            intakeSubsystem.intakeFlap(IntakeSubsystem.IntakeFlapServoState.DOWN);

            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.READY);
            outtakeSubsystem.pivotState(OuttakeSubsystem.OuttakePivotServoState.READY);
            outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
        }
        waitForStart();
        resetTimer();

        while (opModeIsActive())
        {
            globalTimer = GlobalTimer.milliseconds();
            intakeSubsystem.intakeReads(state == autoState.INTAKE || state == autoState.TRANSFER_START);
            outtakeSubsystem.outtakeReads();

            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            autoSequence();
            hardware.update();
            Pose poseEstimate = hardware.drive.getPoseEstimate();
            DashboardUtil.drawRobot(fieldOverlay, poseEstimate.toPose2d());
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
    public void autoSequence()
    {
        switch (state)
        {
            case INTAKE:
                if (((cycle == 0 && trajectories.firstIntakeTrajectory.isFinished()) ||
                        (cycle == 1 && trajectories.firstSampleToHP.isFinished()))
                        && intakeSubsystem.getColorValue() > 800)
                {
                    state = autoState.DEPOSIT_DRIVE;
                    resetTimer();
                    break;
                }
                if (cycle == 0)
                {
                    hardware.drive.followTrajectorySplineHeading(trajectories.firstIntakeTrajectory);
                    if (delay(400))
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                }
                if (cycle == 1)
                {
                    hardware.drive.followTrajectorySplineHeading(trajectories.firstSampleToHP);
                    if (delay(200))
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                }
                break;
            case TRANSFER_START:
                if (delay(600) && intakeSubsystem.slideReached(slideTeleBase))
                {
                    state = autoState.TRANSFER_END;
                    resetTimer();
                    break;
                }
                if (delay(70))
                {
                    intakeClipHoldLogic(slideTeleTransfer, 1); // this controls the intake slides and the clip
                }
                if (intakeSubsystem.slideReached(slideTeleBase))
                {
                    if (delay(100))
                        outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                    if (delay(230))
                    {
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HIGH);
                        outtakeSubsystem.pivotState(OuttakeSubsystem.OuttakePivotServoState.TRANSFER);
                    }
                    if (delay(400))
                    {
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER);
                    }
                } else if (delay(20))
                    outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER_FINISH);
                // naming makes no sense but this makes sure the arm i high when the slides come back
                break;
            case TRANSFER_END:
                // so this is when the thing will grip and we are assuming that the slides are at transfer position
                if (delay(600))
                {
                    intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.OFF);
                    intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.HOLD);
                    state = autoState.DEPOSIT_DRIVE;
                    resetTimer();
                    break;
                }
                intakeClipHoldLogic(slideTeleTransfer, 5); // this controls the intake slides and the clip
                //outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftBasePos); // may be necessary an offset, hopefully not with box tube
                if ((delay(250) && outtakeSubsystem.liftReached(OuttakeSubsystem.liftBasePos) || delay(400)))
                {
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                    if (delay(350))
                    {
                        intakeSubsystem.intakeFlap(IntakeSubsystem.IntakeFlapServoState.TRANSFER);
                    }
                    if (delay(400))
                    {// we wait a bit to to pivot
                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.LOW);
                    }
                    if (delay(440))
                    {
                        outtakeSubsystem.liftToInternalPID(5);
                        if (delay(500))
                            outtakeSubsystem.pivotState(OuttakeSubsystem.OuttakePivotServoState.TRANSFER_FINISH);
                    }
                }
                break;
            case DEPOSIT_DRIVE:
                if ((cycle == 0 && trajectories.secondSampleToHP.isFinished()) ||
                        (cycle == 1 && trajectories.secondDepositTrajectory.isFinished()))
                {
                    cycle++;
                    state = autoState.DROP;
                    resetTimer();
                    break;
                }
                if (cycle == 0)
                    hardware.drive.followTrajectory(trajectories.secondSampleToHP);
                else if (cycle == 1)
                    hardware.drive.followTrajectoryTangentially(trajectories.secondDepositTrajectory, true);
                break;
            case DROP:
                if (delay(600) && dropped)
                {
                    dropped = false;
                    state = cycle == 0 ? autoState.INTAKE : autoState.PARK;
                    resetTimer();
                    break;
                }
                if (!dropped)
                {
                    if (delay(90))
                    {
                        outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftHighBucketPos);
                        outtakeSubsystem.pivotState(OuttakeSubsystem.OuttakePivotServoState.SAMPLE);
                        if (outtakeSubsystem.liftReached(OuttakeSubsystem.liftHighBucketPos)) // this reduces the huge backlash on the arm and improves liftPID
                            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SAMPLE);
                        else
                            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.STRAIGHT);
                    }
                    if (delay(400))
                    {
                        outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                        dropped = true;
                        resetTimer(); // this just like makes it be a new state
                    }
                }
                else
                {
                    if (delay(160))
                    {
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.READY);
                    }
                    else outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.STRAIGHT);

                    outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftBasePos);
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
    public void intakeClipHoldLogic(int slideToPosition, int closeThreshold)
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
    public void outtakeLiftPresets(boolean isSample, boolean isLow, int offSet)
    {
        if (isSample)
        {
            if (isLow) outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftLowBucketPos + offSet);
            else outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftHighBucketPos + offSet);
        }
        else
        {
            if (isLow) outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftLowBarPos + offSet);
            else outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftHighBarPos + offSet);
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
*/
