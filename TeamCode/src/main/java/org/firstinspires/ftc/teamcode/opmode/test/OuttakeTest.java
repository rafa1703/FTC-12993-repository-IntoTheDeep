package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.system.accessory.math.Angles;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

@Config
@TeleOp(name = "Outtake test", group = "Test")
public class OuttakeTest extends LinearOpMode
{
    OuttakeSubsystem outtakeSubsystem;
    IntakeSubsystem intakeSubsystem;
    GeneralHardware hardware;
    FtcDashboard dash = FtcDashboard.getInstance();
    //public static double clawPos = 0, wristPos = 0, pivotPos = 0, armPos = 0;
    public static double intakeLefArmPos = 0, intakeRightArmPos = 0, turretPos = 0, flapPos = 0, clipPos = 0, intakeSpin = 0, intakeSlide = 0;
    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.Red);
        outtakeSubsystem = new OuttakeSubsystem(hardware);
        waitForStart();
        while (opModeIsActive())
        {
            hardware.resetCacheHubs();
            if (gamepad1.a)
            {
                outtakeSubsystem.turretRawControl(-1);

            }
            else outtakeSubsystem.turretRawControl(0);
//            if (gamepad1.a)
//                outtakeSubsystem.turretSpinTo(turretPos);
//            else if (gamepad1.b)
//                outtakeSubsystem.turretSpinTo(turretPos, 1);
//            else if (gamepad1.y)
//                outtakeSubsystem.turretRawControl(0.3);
//            else outtakeSubsystem.turretRawControl(0);
//            //outtakeSubsystem.liftMotorRawControl(1);
//            outtakeSubsystem.armSetPos(armPos);
//            outtakeSubsystem.pivotSetPos(pivotPos);
//            outtakeSubsystem.wristSetPos(wristPos);
//            outtakeSubsystem.clawSetPos(clawPos);
            telemetry.addData("Turret abs angle", outtakeSubsystem.turretAngle);
            telemetry.addData("Turret wrapped angle", Angles.normalizeDegrees(outtakeSubsystem.turretAngle));
            telemetry.addData("Turret position", outtakeSubsystem.turretIncrementalPosition);
            telemetry.addData("Turret intial offset", outtakeSubsystem.initialOffsetPosition);
            telemetry.addData("Turret Target tick", outtakeSubsystem.turretAngleToTicks(turretPos));
            telemetry.addData("Turret target", turretPos);
            telemetry.update();

        }
    }
}
