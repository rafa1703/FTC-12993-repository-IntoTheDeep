package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

@Config
@TeleOp(name = "IntakeTest", group = "Test")
public class IntakeTest extends LinearOpMode
{
    OuttakeSubsystem outtakeSubsystem;
    IntakeSubsystem intakeSubsystem;
    GeneralHardware hardware;
    public static double clawPos = 0, wristPos = 0, railPos = 0, armPos = 0;
    public static double intakeLefArmPos = 0, intakeRightArmPos = 0, chutePos = 0, flapPos = 0, clipPos = 0;
    @Override
    public void runOpMode() throws InterruptedException
    {
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.Red);
        outtakeSubsystem = new OuttakeSubsystem(hardware);
        intakeSubsystem = new IntakeSubsystem(hardware);
        waitForStart();
        while (opModeIsActive())
        {
            intakeSubsystem.intakeFlap(IntakeSubsystem.IntakeFlapServoState.DOWN);
            if (gamepad1.a)
            {
                intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
            }
            if (gamepad1.x)
            {
                intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.OFF);
            }
        }
    }
}
