package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gvf.LocalizerPinpoint;
import org.firstinspires.ftc.teamcode.gvf.odo.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.gvf.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBaseSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;
@TeleOp(group = "Test")
public class PinpointTest extends LinearOpMode
{
    DriveBaseSubsystem drive;
    GoBildaPinpointDriver pinpoint;
    LocalizerPinpoint localizer;
    GeneralHardware hardware;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    @Override
    public void runOpMode() throws InterruptedException
    {
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.RED);
        drive = new DriveBaseSubsystem(hardware);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(inToMM(6.25), inToMM(-3.35));
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();
        localizer = new LocalizerPinpoint(pinpoint);
        waitForStart();
        while (opModeIsActive())
        {
            drive.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();
            Pose pose = localizer.getPoseEstimate();
            Pose predictedPose = localizer.getPredictedPoseEstimate();

            packet.put("x", pose.getX());
            packet.put("y", pose.getY());
            packet.put("heading (deg)", Math.toDegrees(pose.getHeading()));

            //packet.put("Localizer velX", localizer.getVelocity().getX());
            //packet.put("Localizer velY", localizer.getVelocity().getY());

            DashboardUtil.drawRobot(fieldOverlay, predictedPose.toPose2d(), true, "red");
            DashboardUtil.drawRobot(fieldOverlay, pose.toPose2d(), true, "black");

            localizer.update();
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
    public double inToMM(double in)
    {
        return in * 25.28;
    }
}
