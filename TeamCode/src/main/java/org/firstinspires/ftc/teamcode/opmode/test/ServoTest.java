package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

@Config
@TeleOp(name = "ServoTest", group = "Test")
public class ServoTest extends LinearOpMode
{
    OuttakeSubsystem outtakeSubsystem;
    IntakeSubsystem intakeSubsystem;
    GeneralHardware hardware;
    public static double clawPos = 0.879, wristPos = 0.455, railPos = 0.17, armPos = 0.4;
    public static double intakeLefArmPos = 0, intakeRightArmPos = 0, chutePos = 1, flapPos = 0, clipPos = 1;
    @Override
    public void runOpMode() throws InterruptedException
    {
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.Red);
        outtakeSubsystem = new OuttakeSubsystem(hardware);
        intakeSubsystem = new IntakeSubsystem(hardware);

//        clawPos = outtakeSubsystem.cla;
//        wristPos = 0;
//        railPos = 0;
//        armPos = 0;
//        intakeLefArmPos = 0;
//        intakeRightArmPos = 0;
//        chutePos = 0;
//        flapPos = 0;
//        clipPos = 0;
        waitForStart();
        while (opModeIsActive())
        {
            outtakeSubsystem.liftMotorRawControl(0);
            outtakeSubsystem.railSetPos(railPos);
            outtakeSubsystem.armSetPos(armPos);
            outtakeSubsystem.wristSetPos(wristPos);
            outtakeSubsystem.clawSetPos(clawPos);

            intakeSubsystem.armSetPos(intakeRightArmPos, intakeLefArmPos);
            intakeSubsystem.flapSetPos(flapPos);
            intakeSubsystem.clipSetPos(clipPos);
            intakeSubsystem.chuteSetPos(chutePos);
            intakeSubsystem.flapSetPos(flapPos);
//            if (gamepad1.a)
//            {
//                intakeSubsystem.disablePWM(IntakeSubsystem.Servos.FLAP);
//            }
//            if (gamepad1.x)
//            {
//                intakeSubsystem.enablePWM(IntakeSubsystem.Servos.FLAP);
//            }
        }
    }
}
