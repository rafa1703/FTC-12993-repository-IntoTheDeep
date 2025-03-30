package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.gvf.MecanumDrive;
import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.firstinspires.ftc.teamcode.gvf.utils.Vector;
import org.firstinspires.ftc.teamcode.system.accessory.ToggleRisingEdge;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;
import org.firstinspires.ftc.teamcode.system.vision.CameraHardware;
import org.firstinspires.ftc.teamcode.system.vision.InverseKinematics;

@TeleOp(group = "Test")
public class SubIntakeTest extends LinearOpMode
{
    Limelight3A lime;
    IntakeSubsystem intakeSubsystem;
    GeneralHardware hardware;
    InverseKinematics kinematics = new InverseKinematics();
    ToggleRisingEdge toggle = new ToggleRisingEdge();
    LLResult result;
    CameraHardware cameraHardware;
     double slideTarget;

    enum SUB_INTAKE
    {
        FIND,
        BEFORE_INTAKE,
        INTAKE
    }
    SUB_INTAKE state = SUB_INTAKE.FIND;
    Vector sampleDis;
    boolean foundSample;
    double globalTimer, sequenceTimer;
    ElapsedTime GlobalTimer;
    Pose drivePose = new Pose();
    double xOffset = 5;
    @Override
    public void runOpMode() throws InterruptedException
    {
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.RED, true);
        hardware.drive.getLocalizer().setOffSet(new Pose(0, 0, Math.toRadians(0)));

        hardware.drive.setRunMode(MecanumDrive.RunMode.P2P);
        intakeSubsystem = new IntakeSubsystem(hardware);
        cameraHardware = new CameraHardware(hardware);
        cameraHardware.pipelineSwitch(CameraHardware.PipelineType.YELLOW);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        GlobalTimer = new ElapsedTime(System.nanoTime());
        hardware.limelight.start();

        waitForStart();
        while (opModeIsActive())
        {
            globalTimer = GlobalTimer.milliseconds();
            switch (state)
            {
                case FIND:
                    if (foundSample && delay(100))
                    {
                        state = SUB_INTAKE.BEFORE_INTAKE;
//                        Vector offsetVector = new Vector(0, -xOffset); // x is passed in y because the robot is at heading 90 for it to be x
//                        offsetVector = offsetVector.rotated(Math.toRadians(90 - result.getTxNC()) * -1);
//                        drivePose = new Pose(offsetVector.getX(), offsetVector.getY(), Math.toRadians(90 - result.getTxNC())); // sampleDis.getX() + xOffset
                        drivePose = new Pose(0, sampleDis.getX() - 5, Math.toRadians(0));
                        hardware.drive.setTargetPose(drivePose);
                        resetTimer();
                        break;
                    }
                    intakeSubsystem.armSetPos(0.6);
                    intakeSubsystem.intakeTurretSetPos(0.6);
                    result = cameraHardware.getLatestResult();
                    if (result == null) break;

                    foundSample = true;
                    sampleDis = InverseKinematics.distanceToSample(result.getTy(), result.getTx());
                    slideTarget = sampleDis.getY() - 7;
                    telemetry.addData("TX and TY", result.getTxNC() + result.getTyNC());
                    break;
                case BEFORE_INTAKE:
                    if ((delay(300) && hardware.drive.reachedTargetAndHeading(1) && intakeSubsystem.slideReached(slideTarget))
                            || delay(3000))
                    {
                        state = SUB_INTAKE.INTAKE;
                        resetTimer();
                        break;
                    }
//                    hardware.drive.setTargetPose(new Pose(sampleDis.getX() + xOffset, 0, Math.toRadians(90)));
                    intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);
                    intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HALF_DOWN);
                    if (hardware.drive.reachedHeading(1))
                        intakeSubsystem.intakeSlideInternalPID(slideTarget);
                    break;
                case INTAKE:
                    intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
                    if (delay(90)) intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DOWN);
                    if (delay(550)) intakeSubsystem.intakeSlideInternalPID(slideTarget + 2);
                    break;
            }

            hardware.update();
            telemetry.addData("Sample dis", sampleDis.toString());
            telemetry.addData("Target pose", drivePose);
            telemetry.update();
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
