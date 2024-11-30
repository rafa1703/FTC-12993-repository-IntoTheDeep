package org.firstinspires.ftc.teamcode.gvf.opmode;

import static org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware.S;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gvf.trajectories.BezierCurveTrajectorySegment;
import org.firstinspires.ftc.teamcode.gvf.trajectories.Trajectory;
import org.firstinspires.ftc.teamcode.gvf.trajectories.TrajectoryBuilder;
import org.firstinspires.ftc.teamcode.gvf.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;
import org.opencv.core.Point;
@TeleOp(group = "Test")
public class SplineTest extends LinearOpMode
{
    GeneralHardware hardware;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    @Override
    public void runOpMode() throws InterruptedException
    {
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.Red, true);
        hardware.drive.getLocalizer().setPose(new Pose(0, 0, Math.toRadians(0)));
        waitForStart();

        Trajectory spline = new TrajectoryBuilder()
                .addBezierSegment(
                        new Point(0, 0),
                        new Point(24, 0),
                        new Point(24, 24)
                )
                .addFinalPose(new Pose(24, 24, Math.toRadians(90)))
                .build();
        while(opModeIsActive())
        {
            hardware.drive.followTrajectorySplineHeading(spline);
            hardware.update();

            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();
            Pose pose = hardware.drive.getLocalizer().getPoseEstimate();
            Pose predictedPose = hardware.drive.getLocalizer().getPredictedPoseEstimate();

            packet.put("x", pose.getX());
            packet.put("y", pose.getY());
            packet.put("heading (deg)", Math.toDegrees(pose.getHeading()));
            packet.put("xError", Math.abs(spline.getFinalPose().getX() - pose.getX()));
            packet.put("yError", Math.abs(spline.getFinalPose().getY() - pose.getY()));
            packet.put("heading Error (deg)", Math.abs(spline.getFinalPose().getHeading() - pose.getHeading()));
            packet.put("Is spline finished", spline.isFinished());
            DashboardUtil.drawRobot(fieldOverlay, predictedPose.toPose2d(), true, "red");
            DashboardUtil.drawRobot(fieldOverlay, pose.toPose2d(), true, "black");
            DashboardUtil.drawSampledPath(fieldOverlay, spline);

            dashboard.sendTelemetryPacket(packet);
        }
    }
}
