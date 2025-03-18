package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

@Config
@TeleOp(name = "IntakeTest", group = "Test")
public class IntakeTest extends LinearOpMode
{
    OuttakeSubsystem outtakeSubsystem;
    IntakeSubsystem intakeSubsystem;
    GeneralHardware hardware;
    //public static double clawPos = 0, wristPos = 0, railPos = 0, armPos = 0;
   // public static double intakeLefArmPos = 0, intakeRightArmPos = 0, turretPos = 0, flapPos = 0, clipPos = 0, intakeSpin = 0, intakeSlide = 0;
    public static double clip;
    public static double armO = OuttakeSubsystem.OuttakeArmServoState.TRANSFER_BACK.pos,
            wrist = OuttakeSubsystem.OuttakeWristServoState.TRANSFER_BACK.pos,
            armI = IntakeSubsystem.IntakeArmServoState.TRANSFER_BACK.pos;
    public static OuttakeSubsystem.OuttakeClawServoState claw = OuttakeSubsystem.OuttakeClawServoState.INTAKE;
    @Override
    public void runOpMode() throws InterruptedException
    {
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.Red);
        intakeSubsystem = new IntakeSubsystem(hardware);
        outtakeSubsystem = new OuttakeSubsystem(hardware);
        waitForStart();
        while (opModeIsActive())
        {
            hardware.resetCacheHubs();
            intakeSubsystem.intakeReads(true);
            outtakeSubsystem.outtakeReads(true);
            intakeSubsystem.intakeTurretSetPos(armO);
//            intakeSubsystem.armSetPos(1);
//            intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
//            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DOWN);j
//            intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);
//intakeSubsystem.intakeSlideMotorRawControl(1);
            NormalizedRGBA rgba = hardware.colourSensor.getNormalizedColors();
            telemetry.addData("Intake slide current", hardware.intakeSlidesM.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Pos", intakeSubsystem.slidePosition);
            telemetry.addData("Distance", intakeSubsystem.getDistance());
            telemetry.addData("Color alpha(not normalized", hardware.colourSensor.alpha());
            telemetry.addData("Colour", intakeSubsystem.getColorValue());
            telemetry.addData("Colour RGBA", "R: " + rgba.red+ " G: " +rgba.green + " B: " + rgba.blue + " A: "+ rgba.alpha);
            telemetry.addData("Pos hardware",hardware.intakeSlidesM.getCurrentPosition());
            telemetry.update();

        }
    }
}
