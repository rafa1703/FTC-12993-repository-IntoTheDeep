package org.firstinspires.ftc.teamcode.gvf.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gvf.MecanumDrive;
import org.firstinspires.ftc.teamcode.gvf.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

@TeleOp(group = "Tuning")
public class LateralMultiplierTest extends LinearOpMode
{
    GeneralHardware hardware;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    @Override
    public void runOpMode() throws InterruptedException
    {
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.RED, true);

        hardware.drive.setRunMode(MecanumDrive.RunMode.PID);
        hardware.drive.getLocalizer().setPose(new Pose(0, 0, Math.toRadians(0)));
        Pose targetPose = new Pose(0, -50, Math.toRadians(0));
        hardware.drive.setTargetPose(targetPose);
        waitForStart();
        while (opModeIsActive())
        {
            hardware.resetCacheHubs();

            hardware.drive.update();

            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();
            Pose pose = hardware.drive.getPoseEstimate();
            packet.put("x ", pose.getX());
            packet.put("y ", pose.getY());
            packet.put("heading (deg) ", Math.toDegrees(pose.getHeading()));

            packet.put("x error ", Math.abs(targetPose.getX() - pose.getX()));
            packet.put("y error ", Math.abs(targetPose.getY() - pose.getY()));
            packet.put("heading error (deg) ", Math.toDegrees(Math.abs(pose.getHeading() - targetPose.getHeading())));
            packet.put("Lateral multiplier", targetPose.getY() / pose.getY());
            DashboardUtil.drawRobot(fieldOverlay, pose.toPose2d());

            dashboard.sendTelemetryPacket(packet);

            telemetry.update();
        }
    }
}