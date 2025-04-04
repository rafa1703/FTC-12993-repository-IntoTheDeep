package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.system.accessory.math.Angles;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBaseSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

@Config
@TeleOp(name = "Heading lock test", group = "Test")
public class DriveWithHeadingLockTest extends LinearOpMode
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
        telemetry.addData("Heading", 0);
        telemetry.addData("Target", Math.toRadians(-45));
        telemetry.update();
        hardware.imu.resetYaw();
        waitForStart();
        double intakeTarget = 0, outtakeTarget = 0;
        double intake;
        while (opModeIsActive())
        {
            hardware.resetCacheHubs();

            double heading = Angles.normalizeRadians(hardware.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            double target = Math.toRadians(-45);
            driveBase.driveWithPIDHeadingLock(gamepad1.left_stick_y, gamepad1.left_stick_x, target, heading);


            telemetry.addData("Heading", heading);
            telemetry.addData("Target", target);
            telemetry.update();
        }
    }
}
