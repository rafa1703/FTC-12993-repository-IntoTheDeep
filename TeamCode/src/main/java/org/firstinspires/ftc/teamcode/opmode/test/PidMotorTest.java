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
@TeleOp(name = "PidMotorTest", group = "Test")
public class PidMotorTest extends LinearOpMode
{
    public static int outtakeSlidesTarget = 0;
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    DriveBaseSubsystem driveBase;
    GeneralHardware hardware;
    @Override
    public void runOpMode() throws InterruptedException
    {
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.Red);
        intakeSubsystem = new IntakeSubsystem(hardware);
        outtakeSubsystem = new OuttakeSubsystem(hardware);
        driveBase = new DriveBaseSubsystem(hardware);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        double intakeTarget = 0, outtakeTarget = 0;
        double intake = 0;
        while (opModeIsActive())
        {
            driveBase.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            intakeSubsystem.intakeReads(true);
            outtakeSubsystem.outtakeReads();
            if (gamepad1.a) intakeTarget = 18.5;
            if (gamepad1.x) intakeTarget = 0;
            if (gamepad1.y) intakeTarget = 27.3;
            if (gamepad1.b) intake = -0.4;
            else intake = 0;
            if (gamepad1.right_bumper) intake = 1;
            intakeSubsystem.intakeSlideInternalPID(intakeTarget);
            intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
            intakeSubsystem.intakeSpin(intake);
            if (gamepad1.dpad_right) outtakeTarget = 10;
            if (gamepad1.dpad_left) outtakeTarget = 18;
            if (gamepad1.dpad_down) outtakeTarget = 0;
            if (gamepad1.dpad_up) outtakeTarget = 27;
            //outtakeSubsystem.liftToInternalPIDTicks(outtakeSlidesTarget);
            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SAMPLE);
            outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
            //outtakeSubsystem.intakeSlideMotorRawControl(0.7);

            telemetry.addData("Target", intakeTarget);
            telemetry.addData("Intake slides Pos", intakeSubsystem.slidePosition);
            telemetry.addData("Intake slides Pos Inches", intakeSubsystem.ticksToInchesSlidesMotor(intakeSubsystem.slidePosition));
            telemetry.addData("Outtake slides pos", outtakeSubsystem.liftPosition);
            telemetry.addData("Outtake Pos", outtakeSubsystem.ticksToInchesSlidesMotor(outtakeSubsystem.liftPosition));
            telemetry.addData("Outtake target", outtakeTarget);
            telemetry.addData("Color valor", intakeSubsystem.getColorValue());
            telemetry.addData("IsRed", intakeSubsystem.isRed);
            telemetry.update();
      }
    }
}
