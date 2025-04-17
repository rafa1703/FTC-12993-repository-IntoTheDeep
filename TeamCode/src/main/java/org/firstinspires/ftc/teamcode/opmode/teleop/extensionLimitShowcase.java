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
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.RED);
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

            outtakeSubsystem.turretSpinToGains(OuttakeSubsystem.OuttakeTurretState.SPEC_DEPOSIT_BACK);
            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.SPECIMEN_HIGH_AUTO);
            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.SPECIMEN_HIGH_AUTO_SCORE);

            intakeSubsystem.intakeSlideInternalPID(IntakeSubsystem.slideExtensionLimit);
            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DOWN);
            intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);
        }

    }
}
