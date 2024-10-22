package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.system.hardware.DriveBaseSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

@Config
@TeleOp(name = "DriveBaseTest", group = "Test")
public class DriveBaseMotorDirection extends LinearOpMode
{
    DriveBaseSubsystem driveBase;
    public static double
            FL = 0,
            FR = 0,
            BL = 0,
            BR = 0;
    @Override
    public void runOpMode() throws InterruptedException
    {
        driveBase = new DriveBaseSubsystem(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while (opModeIsActive())
        {
            driveBase.motorDirectionTest(FL, FR, BL, BR);
            telemetry.update();
        }
    }
}
