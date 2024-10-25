package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.system.hardware.DriveBaseSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

@TeleOp
public class ArmTest extends LinearOpMode
{
    OuttakeSubsystem outtakeSubsystem;
    IntakeSubsystem intakeSubsystem;
    DriveBaseSubsystem drive;
    @Override
    public void runOpMode() throws InterruptedException
    {
        outtakeSubsystem = new OuttakeSubsystem(hardwareMap, GeneralHardware.Side.Red);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, GeneralHardware.Side.Red);
        drive = new DriveBaseSubsystem(hardwareMap);
        waitForStart();
        //outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.READY);
        outtakeSubsystem.pivotState(OuttakeSubsystem.OuttakePivotServoState.READY);
        outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
        while (opModeIsActive())
        {
            outtakeSubsystem.outtakeReads();
           /* drive.Drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            outtakeSubsystem.outtakeReads();
            if (gamepad1.dpad_down) outtakeSubsystem.pivotState(OuttakeSubsystem.OuttakePivotServoState.READY);
            if (gamepad1.dpad_up) outtakeSubsystem.pivotState(OuttakeSubsystem.OuttakePivotServoState.TRANSFER);
            if (gamepad1.dpad_right) outtakeSubsystem.pivotState(OuttakeSubsystem.OuttakePivotServoState.SAMPLE);

            if (gamepad1.a) outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SAMPLE);
            if (gamepad1.b) outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.READY);
            if (gamepad1.y) outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER);
            if (gamepad1.x) outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.INTAKE);

            if (gamepad1.right_bumper) outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.CLOSE);
            if (gamepad1.left_bumper) outtakeSubsystem.clawState(OuttakeSubsystem.OuttakeClawServoState.OPEN);
            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HIGH);*/
            outtakeSubsystem.intakeSlideMotorRawControl(0);
            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.INTAKE);

            telemetry.addData("OuttakeReached", outtakeSubsystem.liftReached(OuttakeSubsystem.liftHighBucketPos));
            telemetry.addData("Target", OuttakeSubsystem.liftHighBucketPos);
            telemetry.addData("Position", outtakeSubsystem.liftPosition);
            telemetry.update();
        }
    }

    public boolean outtakeLiftHasReachedPresets(boolean isBucket, boolean isLow)
    {
        if (isBucket)
        {
            if (isLow) return outtakeSubsystem.liftReached(OuttakeSubsystem.liftLowBucketPos);
            else return outtakeSubsystem.liftReached(OuttakeSubsystem.liftHighBucketPos);

        }
        else
        {
            if (isLow) return outtakeSubsystem.liftReached(OuttakeSubsystem.liftLowBarPos);
            else return outtakeSubsystem.liftReached(OuttakeSubsystem.liftHighBarPos);
        }
    }
}
