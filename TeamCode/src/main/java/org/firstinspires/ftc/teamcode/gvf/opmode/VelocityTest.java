package org.firstinspires.ftc.teamcode.gvf.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.gvf.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.firstinspires.ftc.teamcode.gvf.utils.Vector;
import org.firstinspires.ftc.teamcode.system.accessory.Log;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBaseSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;
@TeleOp(group = "Test")
public class VelocityTest extends LinearOpMode
{

    GeneralHardware hardware;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    DriveBaseSubsystem drive;
    int step = 0;

    ElapsedTime timer = new ElapsedTime();
    public static double accelerationTime = 500;
    private boolean stopped = false;
    private double velocityAtStop;

    private double deceleration;
    private double deltaTime;

    Log log = new Log("VelocityLog", true);
    @Override
    public void runOpMode() throws InterruptedException
    {
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.RED, true);
        hardware.drive.getLocalizer().setPose(new Pose(0, 0, Math.toRadians(0)));

        waitForStart();
        timer.reset();
        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.startTime();

        while (opModeIsActive() && !isStopRequested())
        {
            hardware.resetCacheHubs();
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();
            Pose pose = hardware.drive.getPoseEstimate();
            Pose predictedPose = hardware.drive.getPredictedPoseEstimate();
            switch (step)
            {
                case 0:
                    if (timer.milliseconds() <= accelerationTime)
                    {
                        hardware.drive.setTargetVector(new Vector(1, 0));
                    } else
                    {
                        step++;
                        timer.reset();
                        velocityAtStop = hardware.drive.getLocalizer().getVelocity().getX();
                        hardware.drive.setTargetVector(new Vector());
                    }
                    break;
                case 1:
                    if (hardware.drive.getLocalizer().getVelocity().getMagnitude() <= 0.1)
                    {
                        step++;
                        deltaTime = timer.seconds();
                        deceleration = velocityAtStop / deltaTime;
                        stopped = hardware.drive.stopped();
                    }
                    break;
            }
            hardware.drive.update();
            packet.put("x", pose.getX());
            packet.put("y", pose.getY());
            packet.put("heading (deg)", Math.toDegrees(pose.getHeading()));

            packet.put("Localizer velX", hardware.drive.getLocalizer().getVelocity().getX());
            packet.put("Localizer velY", hardware.drive.getLocalizer().getVelocity().getY());

            DashboardUtil.drawRobot(fieldOverlay, predictedPose.toPose2d(), true, "red");
            DashboardUtil.drawRobot(fieldOverlay, pose.toPose2d(), true, "black");

            log.addData(hardware.drive.getLocalizer().getVelocity().getX());
            dashboard.sendTelemetryPacket(packet);

            log.update();
            telemetry.update();
        }
        log.close();
    }
}
