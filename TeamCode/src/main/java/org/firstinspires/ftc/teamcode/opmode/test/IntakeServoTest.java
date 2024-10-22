package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
@Config
@TeleOp(name = "IntakeServoTes", group = "Test")
public class IntakeServoTest extends LinearOpMode
{
    ServoImplEx flapS, chuteS, intakeLeftArmS, intakeRightArmS, intakeClipS;

    public static double flapPos = 0, chutePos = 0.1, leftArmPos = 0.1, rightArmPos = 0.1, clipPos = 0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        flapS = hardwareMap.get(ServoImplEx.class, "flapS");
        chuteS = hardwareMap.get(ServoImplEx.class, "chuteS");
        intakeLeftArmS = hardwareMap.get(ServoImplEx.class, "intakeLeftArmS");
        intakeRightArmS = hardwareMap.get(ServoImplEx.class, "intakeRightArmS");
        intakeClipS = hardwareMap.get(ServoImplEx.class, "clipS");
        //intakeLeftArmS.setDirection(Servo.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive())
        {
            flapS.setPosition(flapPos);
            chuteS.setPosition(chutePos);
            intakeLeftArmS.setPosition(leftArmPos);
            intakeRightArmS.setPosition(rightArmPos);
            intakeClipS.setPosition(clipPos);
        }
    }
}
