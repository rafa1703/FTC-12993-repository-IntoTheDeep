package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;


import org.firstinspires.ftc.teamcode.system.hardware.DriveBaseSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

@Config
@TeleOp(name = "IntakePidMotorTest", group = "Test")
public class PidMotorTest extends LinearOpMode
{
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    DriveBaseSubsystem driveBase;
    @Override
    public void runOpMode() throws InterruptedException
    {
      intakeSubsystem = new IntakeSubsystem(hardwareMap, GeneralHardware.Side.Red);
      outtakeSubsystem = new OuttakeSubsystem(hardwareMap, GeneralHardware.Side.Red);
      driveBase = new DriveBaseSubsystem(hardwareMap);
      telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
      waitForStart();
      double intakeTarget = 0, outtakeTarget = 0;
      double intake = 0;
      while (opModeIsActive())
      {
          driveBase.Drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
          intakeSubsystem.intakeReads(false);
          outtakeSubsystem.outtakeReads();
          if (gamepad1.a) intakeTarget = 18.5;
          if (gamepad1.x) intakeTarget = 0;
          if (gamepad1.y) intakeTarget = 27.3;
          if (gamepad1.b) intake = -0.4;
          else intake = 0;
          if (gamepad1.right_bumper) intake = 1;
          //intakeSubsystem.intakeSlideMotorRawControl(0.5);
          intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.LOW);
          intakeSubsystem.intakeChute(IntakeSubsystem.IntakeChuteServoState.UP);
          intakeSubsystem.intakeFlap(IntakeSubsystem.IntakeFlapServoState.DOWN);
          intakeSubsystem.intakeSlideInternalPID(intakeTarget);
          intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.OPEN);
          intakeSubsystem.intakeSpin(intake);
          if (gamepad1.dpad_right) intakeTarget = 200;
          if (gamepad1.dpad_left) intakeTarget = 400;
          if (gamepad1.dpad_down) intakeTarget = 0;
          if (gamepad1.dpad_up) intakeTarget = 800;
          //outtakeSubsystem.liftToInternalPID(outtakeTarget);
          outtakeSubsystem.intakeSlideMotorRawControl(0);

          telemetry.addData("Target", intakeTarget);
          telemetry.addData("Intake slides Pos", intakeSubsystem.slidePosition);
          telemetry.addData("Intake slides Pos Inches", intakeSubsystem.ticksToInchesSlidesMotor(intakeSubsystem.slidePosition));
          telemetry.addData("Outtake slides pos", outtakeSubsystem.liftPosition);
          telemetry.update();
      }
    }
}
