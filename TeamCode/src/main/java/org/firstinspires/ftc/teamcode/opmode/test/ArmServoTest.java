package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@TeleOp(name = "ArmServoTest", group = "Test")
public class ArmServoTest extends LinearOpMode
{
    ServoImplEx clawS, pivotS, leftArmS, rightArmS;
    public static double clawPos = 0, pivotPos = 0.27, leftArmPos = 0, rightArmPos = 0;
    @Override
    public void runOpMode() throws InterruptedException
    {
        clawS = hardwareMap.get(ServoImplEx.class, "clawS");
        pivotS = hardwareMap.get(ServoImplEx.class, "pivotS");
        leftArmS = hardwareMap.get(ServoImplEx.class, "outtakeLeftArmS");
        rightArmS = hardwareMap.get(ServoImplEx.class, "outtakeRightArmS");

        clawS.setDirection(Servo.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive())
        {
            clawS.setPosition(clawPos);
            pivotS.setPosition(pivotPos);
            //leftArmS.setPosition(leftArmPos);
            //rightArmS.setPosition(rightArmPos);
        }
    }
}
