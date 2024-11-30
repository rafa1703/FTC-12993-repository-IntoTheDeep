package org.firstinspires.ftc.teamcode.opmode.auto;

import android.graphics.Path;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gvf.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBaseSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

@TeleOp(group = "Tuning")
public class PointsTest extends LinearOpMode
{
    GeneralHardware hardware;
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    DriveBaseSubsystem drive;
    @Override
    public void runOpMode() throws InterruptedException
    {
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.Red, true);
        hardware.drive.getLocalizer().setOffSet(new Pose(-7.2, 62.5, -Math.toRadians(90)));
        drive = new DriveBaseSubsystem(hardware);
        intakeSubsystem = new IntakeSubsystem(hardware);
        outtakeSubsystem = new OuttakeSubsystem(hardware);
        while (!isStarted())
        {
            outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
        }

        waitForStart();
        while (opModeIsActive())
        {
            hardware.resetCacheHubs();
            hardware.drive.getLocalizer().update();
            drive.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();
            Pose pose = hardware.drive.getPoseEstimate();
            Pose predictedPose = hardware.drive.getPredictedPoseEstimate();

            if (true)
            {
                outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.SPECIMEN_HIGH);
                outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN);
                outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
                outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SPECIMEN);

                intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HIGH);

            }
            else
            {
                outtakeSubsystem.railState(OuttakeSubsystem.OuttakeRailServoState.INTAKE);
                outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.INTAKE);
                outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
                outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.INTAKE);

            }
            packet.put("x", pose.getX());
            packet.put("y", pose.getY());
            packet.put("heading (deg)", Math.toDegrees(pose.getHeading()));

            packet.put("Localizer velX", hardware.drive.getLocalizer().getVelocity().getX());
            packet.put("Localizer velY", hardware.drive.getLocalizer().getVelocity().getY());

            DashboardUtil.drawRobot(fieldOverlay, predictedPose.toPose2d(), true, "red");
            DashboardUtil.drawRobot(fieldOverlay, pose.toPose2d(), true, "black");


            dashboard.sendTelemetryPacket(packet);

            telemetry.update();
        }
    }
}
