package org.firstinspires.ftc.teamcode.gvf.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gvf.Localizer;
import org.firstinspires.ftc.teamcode.gvf.MecanumDrive;
import org.firstinspires.ftc.teamcode.gvf.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.firstinspires.ftc.teamcode.gvf.utils.Vector;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;
@TeleOp(group = "Test")
public class GlidingTest extends LinearOpMode
{
    GeneralHardware hardware;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    @Override
    public void runOpMode() throws InterruptedException
    {
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.Red, true);
        hardware.startThreads(this);
        hardware.drive.setRunMode(MecanumDrive.RunMode.Vector);
        hardware.drive.setTargetVector(new Vector(1, 0));
        waitForStart();
        while (opModeIsActive())
        {
            hardware.resetCacheHubs();

            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();
            Pose pose = hardware.drive.getPoseEstimate();
            Pose predictedPose = hardware.drive.getPredictedPoseEstimate();
            packet.put("x ", pose.getX());
            packet.put("y ", pose.getY());
            packet.put("heading (deg) ", Math.toDegrees(pose.getHeading()));
            packet.put("Pred x ", predictedPose.getX());
            packet.put("Pred y ", predictedPose.getY());
            packet.put("Pred heading (deg) ", Math.toDegrees(predictedPose.getHeading()));
            packet.put("Gliding vector", hardware.drive.getLocalizer().getGlideDelta());
            packet.put("Velocity vector", hardware.drive.getLocalizer().getVelocity());
            DashboardUtil.drawRobot(fieldOverlay, predictedPose.toPose2d(), true, "red");
            DashboardUtil.drawRobot(fieldOverlay, pose.toPose2d(), true, "blue");
            hardware.drive.update();
            dashboard.sendTelemetryPacket(packet);

            telemetry.update();
        }
    }
}
