package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBaseSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

@Config
@TeleOp(name = "Imu Test", group = "Test")
public class ImuTest extends LinearOpMode
{
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    DriveBaseSubsystem driveBase;
    GeneralHardware hardware;
    @Override
    public void runOpMode() throws InterruptedException
    {
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.RED);
        intakeSubsystem = new IntakeSubsystem(hardware);
        outtakeSubsystem = new OuttakeSubsystem(hardware);
        driveBase = new DriveBaseSubsystem(hardware);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        IMU imu;
        imu = hardware.imu;
        imu.resetYaw();
        while (opModeIsActive())
        {
            hardware.resetCacheHubs();
            driveBase.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            intakeSubsystem.intakeTurretSetAngle(90);
            telemetry.addData("Heading ", imu.getRobotYawPitchRollAngles());
            telemetry.addData("Angular vel ", imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate);
            telemetry.update();
      }
    }
}
