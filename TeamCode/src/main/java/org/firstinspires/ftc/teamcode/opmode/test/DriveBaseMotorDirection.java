package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sun.tools.javac.jvm.Gen;

import org.firstinspires.ftc.teamcode.system.hardware.DriveBaseSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

@Config
@TeleOp(name = "DriveBaseTest", group = "Test")
public class DriveBaseMotorDirection extends LinearOpMode
{
    DriveBaseSubsystem driveBase;
    GeneralHardware hardware;
    OuttakeSubsystem outtakeSubsystem;
    public static double
            FL = 0,
            FR = 0,
            BL = 0,
            BR = 0,
            lift = 0;
    @Override
    public void runOpMode() throws InterruptedException
    {
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.Red);
        driveBase = new DriveBaseSubsystem(hardware);
        outtakeSubsystem = new OuttakeSubsystem(hardware);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while (opModeIsActive())
        {

//            driveBase.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            driveBase.motorDirectionTest(FL, FR, BL, BR);
            outtakeSubsystem.liftMotorRawControl(lift);
            telemetry.update();
        }
    }
}
