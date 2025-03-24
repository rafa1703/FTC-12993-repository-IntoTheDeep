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

@Config
@TeleOp(name = "ClimbTest", group = "Test")
public class ClimbTest extends LinearOpMode
{
    public static double motorPowers = 0;
    public static OuttakeSubsystem.OuttakeLockServoState lock = OuttakeSubsystem.OuttakeLockServoState.OPEN;
    public static double liftPow = 0, ptoPos = 0;
    public static DriveBaseSubsystem.PTOState pto = DriveBaseSubsystem.PTOState.IN;
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

//        intakeSubsystem = new IntakeSubsystem(hardware);
        outtakeSubsystem = new OuttakeSubsystem(hardware);
        driveBase = new DriveBaseSubsystem(hardware);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();

        while (opModeIsActive())
        {
            driveBase.PTOState(pto);
            driveBase.ptoMotorsSetPower(motorPowers);
//            driveBase.PTOSetPosition(ptoPos);
            outtakeSubsystem.lockServoState(lock);
            outtakeSubsystem.liftMotorRawControl(liftPow);
            telemetry.update();
        }
    }
}
