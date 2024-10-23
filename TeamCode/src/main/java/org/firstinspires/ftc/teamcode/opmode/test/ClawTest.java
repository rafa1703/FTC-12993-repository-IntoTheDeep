package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;
@TeleOp
@Config
public class ClawTest extends LinearOpMode
{
    OuttakeSubsystem outtakeSubsystem;
    //IntakeSubsystem intakeSubsystem;
    @Override
    public void runOpMode() throws InterruptedException
    {
        outtakeSubsystem = new OuttakeSubsystem(hardwareMap, GeneralHardware.Side.Red);
        //intakeSubsystem = new IntakeSubsystem(hardwareMap, GeneralHardware.Side.Red);
        waitForStart();

        while (opModeIsActive())
        {
           if (gamepad1.a) outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
           if (gamepad1.y) outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
        }
    }
}
