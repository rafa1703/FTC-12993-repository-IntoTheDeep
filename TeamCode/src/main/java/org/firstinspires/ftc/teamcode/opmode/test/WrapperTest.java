package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers.MotorFoda;
import org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers.MotorPika;
import org.firstinspires.ftc.teamcode.system.hardware.robot.wrappers.ServoPika;

@TeleOp(group = "Test")
public class WrapperTest extends LinearOpMode
{
    MotorPika motor;
    ServoPika servo;
    @Override
    public void runOpMode() throws InterruptedException
    {
        motor = new MotorPika(hardwareMap.get(DcMotorEx.class, "Intake"));
        servo = new ServoPika(hardwareMap.get(ServoImplEx.class, "clipS"));

        waitForStart();
        while (opModeIsActive())
        {
            if (gamepad1.a) motor.setPower(1.0);
            if (gamepad1.b) motor.setPower(0);
            if (gamepad1.x) servo.setPosition(1);
            if (gamepad1.y) servo.setPosition(0.7);
        }
    }
}
