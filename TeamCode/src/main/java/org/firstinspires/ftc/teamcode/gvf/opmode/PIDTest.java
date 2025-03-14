package org.firstinspires.ftc.teamcode.gvf.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gvf.MecanumDrive;
import org.firstinspires.ftc.teamcode.gvf.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;
@TeleOp(group = "Tuning")
public class PIDTest extends LinearOpMode
{
    GeneralHardware hardware;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    IntakeSubsystem intakeSubsystem;
    @Override
    public void runOpMode() throws InterruptedException
    {
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.Red, true);
        intakeSubsystem = new IntakeSubsystem(hardware);
        hardware.drive.setRunMode(MecanumDrive.RunMode.P2P); // works exactly like p2p but with like the actual pose
//        hardware.drive.getLocalizer().setPose(new Pose(0, 0, Math.toRadians(0)));
        hardware.drive.getLocalizer().setOffSet(new Pose(0, 0, Math.toRadians(0)));
        Pose targetPose = new Pose(0, 0, Math.toRadians(180));
        hardware.drive.setTargetPose(targetPose);
        while (!isStarted())
        {
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();
            Pose pose = hardware.drive.getPoseEstimate();

            packet.put("x ", pose.getX());
            packet.put("y ", pose.getY());
            packet.put("heading (deg) ", Math.toDegrees(pose.getHeading()));

            packet.put("x error ", Math.abs(targetPose.getX() - pose.getX()));
            packet.put("y error ", Math.abs(targetPose.getY() - pose.getY()));
            packet.put("heading error (deg) ", Math.toDegrees(Math.abs(pose.getHeading() - targetPose.getHeading())));
            dashboard.sendTelemetryPacket(packet);

            telemetry.update();
        }
        waitForStart();
        while (opModeIsActive())
        {
            hardware.update();
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();
            Pose pose = hardware.drive.getPoseEstimate();
            Pose predictedPose = hardware.drive.getPredictedPoseEstimate();
            packet.put("x ", pose.getX());
            packet.put("y ", pose.getY());
            packet.put("heading (deg) ", Math.toDegrees(pose.getHeading()));

            packet.put("x error ", Math.abs(targetPose.getX() - pose.getX()));
            packet.put("y error ", Math.abs(targetPose.getY() - pose.getY()));
            packet.put("heading error (deg) ", Math.toDegrees(Math.abs(pose.getHeading() - targetPose.getHeading())));

            DashboardUtil.drawRobot(fieldOverlay, pose.toPose2d(), true);
            //DashboardUtil.drawRobot(fieldOverlay, predictedPose.toPose2d(), true);

            hardware.drive.update();
            dashboard.sendTelemetryPacket(packet);

            telemetry.update();
        }
    }
}
