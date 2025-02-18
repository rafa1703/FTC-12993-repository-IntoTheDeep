package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.system.accessory.math.Angles;
import org.firstinspires.ftc.teamcode.system.hardware.DriveBaseSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;
import org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers.MotorPika;

@Config
@TeleOp(name = "TurretTest", group = "Test")
public class TurretTest extends LinearOpMode
{
    DriveBaseSubsystem driveBase;
    GeneralHardware hardware;
    MotorPika turretM;
    AnalogInput turretEncoder;
    public static double vol = 3.227;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.Red);
        turretM = new MotorPika(hardwareMap.get(DcMotorEx.class, "turretM"));
        turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
        driveBase = new DriveBaseSubsystem(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while (opModeIsActive())
        {
            driveBase.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            double angle = turretEncoder.getVoltage() / vol * 360;
            telemetry.addData("Turret angle", angle);
            telemetry.addData("Encoder voltage", turretEncoder.getVoltage());
            telemetry.addData("Encoder max voltage", turretEncoder.getMaxVoltage());
            telemetry.addData("Wrapped angle", Angles.normalizeDegrees(angle));
            if (gamepad1.a)
            {
                turretM.setPower(1);
            }
            else if (gamepad1.b)
            {
                turretM.setPower(0.5);
            }
            else turretM.setPower(0);
         //   driveBase.motorDirectionTest(FL, FR, BL, BR);
            telemetry.update();
        }
    }
}
