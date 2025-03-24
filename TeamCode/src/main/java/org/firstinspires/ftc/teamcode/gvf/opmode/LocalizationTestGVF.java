package org.firstinspires.ftc.teamcode.gvf.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gvf.LocalizerCustomVel;
import org.firstinspires.ftc.teamcode.gvf.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBaseSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

@TeleOp(group = "Tuning")
public class LocalizationTestGVF extends LinearOpMode
{
    GeneralHardware hardware;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    DriveBaseSubsystem drive;
    @Override
    public void runOpMode() throws InterruptedException
    {
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.RED, true);
        drive = new DriveBaseSubsystem(hardware);
        LocalizerCustomVel localizer = new LocalizerCustomVel(hardware);
        waitForStart();
        while (opModeIsActive())
        {
            hardware.resetCacheHubs();
            localizer.update();
            //hardware.drive.getLocalizer().update();
            drive.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();
            Pose pose = localizer.getPoseEstimate();
            Pose predictedPose = localizer.getPredictedPoseEstimate();

            packet.put("x", pose.getX());
            packet.put("y", pose.getY());
            packet.put("heading (deg)", Math.toDegrees(pose.getHeading()));

            packet.put("Localizer velX", localizer.getVelocity().getX());
            packet.put("Localizer velY", localizer.getVelocity().getY());

            DashboardUtil.drawRobot(fieldOverlay, predictedPose.toPose2d(), true, "red");
            DashboardUtil.drawRobot(fieldOverlay, pose.toPose2d(), true, "black");


            dashboard.sendTelemetryPacket(packet);

            telemetry.update();
        }
    }
}
