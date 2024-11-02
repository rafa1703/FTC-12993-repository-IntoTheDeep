
package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTeleBase;
import static org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem.slideTeleTransfer;
import static org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware.S;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.gvf.MecanumDrive;
import org.firstinspires.ftc.teamcode.gvf.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

@Autonomous(name = "RED CLOSE 1 +2", group = "RedClose")
public class redCloseAutoPreload extends LinearOpMode
{

    enum autoState {
        PRELOAD_DEPOSIT,
        INTAKE,
        TRANSFER_START,
        TRANSFER_END,
        DEPOSIT_DRIVE,
        DROP,
        PARK,
        IDLE
    }
    ElapsedTime GlobalTimer;
    autoState state = autoState.PRELOAD_DEPOSIT;
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
        hardware.drive.setRunMode(MecanumDrive.RunMode.PID);
        hardware.drive.getLocalizer().setPose(new Pose(-3.5, -62.3 * S, Math.toRadians(90 * S)));
        hardware.startThreads(this);
        intakeSubsystem = new IntakeSubsystem(hardware);
        outtakeSubsystem = new OuttakeSubsystem(hardware);
        GlobalTimer = new ElapsedTime(System.nanoTime());
        globalTimer = GlobalTimer.milliseconds();
        resetTimer();

        while (!isStarted())
        {

            intakeSubsystem.intakeChute(IntakeSubsystem.IntakeChuteServoState.UP);
            intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.HOLD);

            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER);
            if (delay(300)) outtakeSubsystem.pivotState(OuttakeSubsystem.OuttakePivotServoState.TRANSFER);
            else outtakeSubsystem.pivotState(OuttakeSubsystem.OuttakePivotServoState.READY);

            if (delay(1500))
            {
                outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                intakeSubsystem.intakeFlap(IntakeSubsystem.IntakeFlapServoState.TRANSFER);
            }
            else
            {
                outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                intakeSubsystem.intakeFlap(IntakeSubsystem.IntakeFlapServoState.DOWN);
            }
            if (delay(2000)) intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.LOW);
            else intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.LOW);
            globalTimer = GlobalTimer.milliseconds();
        }
        waitForStart();
        globalTimer = GlobalTimer.milliseconds();
        resetTimer();
        while (opModeIsActive())
        {
            hardware.update();
            globalTimer = GlobalTimer.milliseconds();
            intakeSubsystem.intakeReads(state == autoState.INTAKE || state == autoState.TRANSFER_START);
            outtakeSubsystem.outtakeReads();

            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            autoSequence();
            hardware.drive.update();
            Pose poseEstimate = hardware.drive.getPoseEstimate();
            DashboardUtil.drawRobot(fieldOverlay, poseEstimate.toPose2d(), true);
            dashboard.sendTelemetryPacket(packet);
            telemetry.addData("State", state);
            telemetry.update();
        }
    }
    public void autoSequence()
    {
        switch (state)
        {
            case PRELOAD_DEPOSIT:
                if (delay(400) && hardware.drive.reachedTarget(2) && hardware.drive.stopped())
                {
                    state = autoState.DROP;
                    resetTimer();
                    break;
                }
                outtakeSubsystem.pivotState(OuttakeSubsystem.OuttakePivotServoState.TRANSFER_FINISH);
                outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER_FINISH);
                outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                hardware.drive.setTargetPose(new Pose(-54.5, -56.5 * S, Math.toRadians(45 * S))); // this is the same drop pos as the other auto
                break;
            case INTAKE:
                if (intakeSubsystem.getColorValue() > 800)
                {
                    state = autoState.TRANSFER_START;
                    resetTimer();
                    break;
                }
                if (delay(20))
                {
                    intakeSubsystem.intakeFlap(IntakeSubsystem.IntakeFlapServoState.DOWN);
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                }
                if (cycle == 1)
                {
                    hardware.drive.setTargetPose(new Pose(-43.8, -39.8 * S, Math.toRadians(90 * S)));
                    if (hardware.drive.reachedTarget(2))
                    {
                        Pose intakePose = new Pose(-43.8, (-39.8 + 9) * S, Math.toRadians(90 * S));
                        hardware.drive.setTargetPose(intakePose);
                    }
                    if (delay(200))
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                }
                if (cycle == 2)
                {
                    hardware.drive.setTargetPose(new Pose(-53.4, -39.8 * S, Math.toRadians(90 * S)));
                    if (hardware.drive.reachedTarget(2))
                    {
                        Pose intakePose = new Pose(-53.8, (-39.8 + 9) * S, Math.toRadians(90 * S));
                        hardware.drive.setTargetPose(intakePose);
                    }
                    if (delay(200))
                        intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                }
                break;
            case TRANSFER_START:
                if (delay(650) && intakeSubsystem.isSlidesAtBase())
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
                        //intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HIGH);
                        outtakeSubsystem.pivotState(OuttakeSubsystem.OuttakePivotServoState.TRANSFER);
                    }
                    if (delay(400))
                    {
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER);
                    }
                } else if (delay(20))
                    //outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER_FINISH);
                // naming makes no sense but this makes sure the arm i high when the slides come back
                break;
            case TRANSFER_END:
                // so this is when the thing will grip and we are assuming that the slides are at transfer position
                if (delay(700))
                {
                    intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.HOLD);
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.LOW);
                    state = autoState.DEPOSIT_DRIVE;
                    resetTimer();
                    break;
                }
                intakeClipHoldLogic(slideTeleTransfer, 5); // this controls the intake slides and the clip
                //outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftBasePos); // may be necessary an offset, hopefully not with box tube
                if (delay(150)) intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.OFF);
                if (delay(350))
                {
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                    if (delay(450))
                    {
                        intakeSubsystem.intakeFlap(IntakeSubsystem.IntakeFlapServoState.TRANSFER);
                    }
                    if (delay(500))
                    {
                        outtakeSubsystem.liftToInternalPID(5);
                        if (delay(540))
                            outtakeSubsystem.pivotState(OuttakeSubsystem.OuttakePivotServoState.TRANSFER_FINISH);
                    }
                }
                break;
            case DEPOSIT_DRIVE:
                if (
                        ((cycle == 0 && hardware.drive.reachedTarget(2)) ||
                                (cycle == 1 && hardware.drive.reachedTarget(2)) ||
                                (cycle == 2 && hardware.drive.reachedTarget(2))
                                )
                                && delay(600) && hardware.drive.stopped())
                {
                    state = autoState.DROP;
                    resetTimer();
                    break;
                }
                if (delay(100))
                {
                    if (cycle == 1)
                        hardware.drive.setTargetPose(new Pose(-53.5, -55.5 * S, Math.toRadians(45 * S)));
                    else if (cycle == 2)
                        hardware.drive.setTargetPose(new Pose(-53.5, -55.5 * S, Math.toRadians(45 * S)));
                }
                break;
            case DROP:
                if (delay(1000) && dropped)
                {
                    dropped = false;
                    state = cycle == 2 ? autoState.PARK : autoState.INTAKE;
                    cycle++;
                    resetTimer();
                    break;
                }
                if (!dropped)
                {
                    outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE); // this shouldnt be necessary
                    if (delay(90))
                    {
                        outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftHighBucketPos);
                        outtakeSubsystem.pivotState(OuttakeSubsystem.OuttakePivotServoState.SAMPLE);
                        if (delay(400)) outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SAMPLE);
                        else outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.STRAIGHT);
                    }
                    if (delay(2500))
                    {
                        outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                        dropped = true;
                        resetTimer(); // this just like makes it be a new state
                    }
                }
                else // everything here is ran as it was a new state
                {
                    if (delay(700)) // this is 300 after dropped
                    {
                        outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.READY);
                        outtakeSubsystem.pivotState(OuttakeSubsystem.OuttakePivotServoState.READY);
                        outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                    }
                    if (delay(900))
                    {
                        outtakeSubsystem.liftToInternalPID(OuttakeSubsystem.liftBasePos);
                        hardware.drive.setTargetPose(new Pose(-50, -50 * S, Math.toRadians(45 * S)));
                    }

                }
                break;
            case PARK:
                if (hardware.drive.reachedTarget(2) && delay(200))
                {
                    state = autoState.IDLE;
                    resetTimer();
                    break;
                }
                hardware.drive.setTargetPose(new Pose(53, -55 * S, Math.toRadians(180 * S)));
                break;
            case IDLE: // we idle here duuhhh
                break;
        }
        if (delay(7500) && state == autoState.INTAKE)// if the sample is stuck we just park
        {
            state = autoState.PARK;
            resetTimer();
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
