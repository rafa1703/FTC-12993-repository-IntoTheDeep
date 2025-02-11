package org.firstinspires.ftc.teamcode.gvf.opmode;


import static org.firstinspires.ftc.teamcode.system.accessory.math.Angles.normalizeRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.gvf.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.firstinspires.ftc.teamcode.gvf.utils.Vector;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;


@Config
@TeleOp(name = "Deceleration Accuracy Tuner", group="Test")
public class DecelAccTest extends LinearOpMode
{

    FtcDashboard dash;

    GeneralHardware hardware;

    ElapsedTime timer = new ElapsedTime();

    public static double accelerationTime = 1;
    private boolean stopped = false;
    private double velocityAtStop;

    private double deceleration;
    private double deltaTime;
    private Pose predictedPose;
    private Pose endPose;
    private double errorDistance;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    IntakeSubsystem intakeSubsystem;

    int step = 0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.Red, true);
        intakeSubsystem = new IntakeSubsystem(hardware);

        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());
        while (!isStarted())
        {
            intakeSubsystem.intakeSlideInternalPID(18.5);
        }
        waitForStart();
        hardware.drive.getLocalizer().setPose(new Pose(0, 0 , Math.toRadians(0)));

        timer.reset();

        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.startTime();

        while (opModeIsActive() && !isStopRequested())
        {
            hardware.drive.getLocalizer().setHeadingDecelInterpolationOnSlidePos(18.5 / 18.5);
            hardware.update();
            switch (step)
            {
                case 0:
                    if (timer.seconds() <= accelerationTime)
                    {
                        hardware.drive.setTargetVector(new Vector(0, 0, 1));
                    } else
                    {
                        step++;
                        timer.reset();
                        predictedPose = hardware.drive.getPredictedPoseEstimate();
                        hardware.drive.setTargetVector(new Vector());
                    }
                    break;
                case 1:
                    if (hardware.drive.getLocalizer().getVelocity().getMagnitude() <= 0.1)
                    {
                        step++;
                        deltaTime = timer.seconds();
                        endPose = hardware.drive.getPoseEstimate();
                        stopped = hardware.drive.stopped();
                        Vector end = new Vector(endPose.toPoint());
                        Vector predicted = new Vector(predictedPose.toPoint());
                        errorDistance = normalizeRadians(predictedPose.getHeading() - endPose.getHeading());//predicted.subtract(end).getMagnitude();

                    }
                    break;
            }
            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HIGH);
            intakeSubsystem.intakeSlideInternalPID(18.5);
            // 2.62 y decel, 87.68 x decel
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            hardware.drive.update();
            telemetry.addData("pose", hardware.drive.getPoseEstimate());
            telemetry.addData("Predicted pose", predictedPose);
            telemetry.addData("Error magnitude", errorDistance);
            telemetry.addData("Step", step);
            telemetry.addData("Timer", timer.seconds());
            telemetry.addData("Deceleration", deceleration);
            telemetry.addData("Delta time", deltaTime);
            telemetry.addData("Velocity at stop", velocityAtStop);
            telemetry.addData("Time since stop", timer.seconds());
            telemetry.addData("Stopped", stopped);
            telemetry.addData("Velocity x", hardware.drive.getLocalizer().getVelocity().getX());
            telemetry.addData("Velocity y", hardware.drive.getLocalizer().getVelocity().getY());
            telemetry.addData("Imu angle", hardware.drive.getLocalizer().getHeading());

            if (predictedPose != null) DashboardUtil.drawRobot(fieldOverlay, predictedPose.toPose2d(), true, "red");
            DashboardUtil.drawRobot(fieldOverlay, hardware.drive.getPoseEstimate().toPose2d(), true, "black");
            dashboard.sendTelemetryPacket(packet);

            telemetry.update();
        }
    }
}
