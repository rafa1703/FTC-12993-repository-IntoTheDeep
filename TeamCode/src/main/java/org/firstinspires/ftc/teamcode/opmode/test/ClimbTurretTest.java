package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.system.hardware.DriveBaseSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

@TeleOp(name = "ClimbTurretTest", group = "Test")
public class ClimbTurretTest extends LinearOpMode
{
    public static double motorPowers = 0;
    public static OuttakeSubsystem.OuttakeLockServoState lock = OuttakeSubsystem.OuttakeLockServoState.OPEN;
    public static double liftPow = 0, ptoPos = 0;
    public static DriveBaseSubsystem.PTOState pto = DriveBaseSubsystem.PTOState.IN;
    public static double leftClimbServoPow, rightClimbServoPow;
    // in 2900
    // into the thing 440
    // hook onto other thing 100
    // climbing 4000

    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    DriveBaseSubsystem driveBase;
    GeneralHardware hardware;
    @Override
    public void runOpMode() throws InterruptedException
    {
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.RED);

        intakeSubsystem = new IntakeSubsystem(hardware);
        outtakeSubsystem = new OuttakeSubsystem(hardware);
        driveBase = new DriveBaseSubsystem(hardware);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.HOLD);
        outtakeSubsystem.lockServoState(OuttakeSubsystem.OuttakeLockServoState.LOCKED);
        waitForStart();
        boolean back = true;
        while (opModeIsActive())
        {
            hardware.resetCacheHubs();
            intakeSubsystem.intakeClip(IntakeSubsystem.IntakeClipServoState.HOLD);
            outtakeSubsystem.lockServoState(OuttakeSubsystem.OuttakeLockServoState.LOCKED);
            outtakeSubsystem.outtakeReads();
            if (gamepad1.a) back = true;
            else if (gamepad1.b) back = false;
            driveBase.drive(0, 0, gamepad1.right_stick_x);
            if (back)
            {
                outtakeSubsystem.turretSpinToGains(OuttakeSubsystem.OuttakeTurretState.TRANSFER_BACK);
            }
            else outtakeSubsystem.turretSpinToGains(OuttakeSubsystem.OuttakeTurretState.TRANSFER_FRONT);
            telemetry.update();
        }
    }
}
