package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.system.hardware.DriveBaseSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;
@TeleOp(group = "Test")
public class extensionLimitShowcase extends LinearOpMode
{
    GeneralHardware hardware;
    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    DriveBaseSubsystem drive;

    @Override
    public void runOpMode() throws InterruptedException
    {
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.Red);
        intakeSubsystem = new IntakeSubsystem(hardware);
        outtakeSubsystem = new OuttakeSubsystem(hardware);
        drive = new DriveBaseSubsystem(hardware);
        drive.setUpZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        while (opModeIsActive())
        {
            hardware.resetCacheHubs();
            intakeSubsystem.intakeReads(false);
            outtakeSubsystem.outtakeReads();
            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.INTAKE);
            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.INTAKE);
            intakeSubsystem.intakeSlideInternalPID(18.5);
        }

    }
}
