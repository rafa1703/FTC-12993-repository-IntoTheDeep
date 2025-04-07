package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.system.hardware.DriveBaseSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

@Config
@TeleOp(name = "IntakeTest", group = "Test")
public class IntakeTest extends LinearOpMode
{
    OuttakeSubsystem outtakeSubsystem;
    IntakeSubsystem intakeSubsystem;
    DriveBaseSubsystem drive;
    GeneralHardware hardware;
    //public static double clawPos = 0, wristPos = 0, railPos = 0, armPos = 0;
   // public static double intakeLefArmPos = 0, intakeRightArmPos = 0, turretPos = 0, flapPos = 0, clipPos = 0, intakeSpin = 0, intakeSlide = 0;
    public static double clip;
    public static double armO = OuttakeSubsystem.OuttakeArmServoState.TRANSFER_BACK.pos,
            wrist = OuttakeSubsystem.OuttakeWristServoState.TRANSFER_BACK.pos,
            armI = IntakeSubsystem.IntakeArmServoState.HP_DEPOSIT.pos,
            turretI = IntakeSubsystem.IntakeTurretServoState.HP_DEPOSIT.pos;
    public static OuttakeSubsystem.OuttakeClawServoState claw = OuttakeSubsystem.OuttakeClawServoState.INTAKE;
    public static double pto = 0.5;
    @Override
    public void runOpMode() throws InterruptedException
    {
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.RED);
        intakeSubsystem = new IntakeSubsystem(hardware);
        outtakeSubsystem = new OuttakeSubsystem(hardware);
        drive = new DriveBaseSubsystem(hardware);
        waitForStart();
        while (opModeIsActive())
        {
            hardware.resetCacheHubs();
            intakeSubsystem.intakeReads(true);
            outtakeSubsystem.outtakeReads(true);
            intakeSubsystem.clipSetPos(pto);
            drive.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
//            drive.PTOSetPosition(pto);

            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DOWN);
            intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);
            if (gamepad1.a) intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.REVERSE);
            else intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);

//            intakeSubsystem.intakeSlideInternalPID(23.2);
            telemetry.addData("Filter NEUTRAL", intakeSubsystem.checkColour(IntakeSubsystem.IntakeFilter.NEUTRAL));
            telemetry.addData("Filter SIDE ONLY", intakeSubsystem.checkColour(IntakeSubsystem.IntakeFilter.SIDE_ONLY));
            telemetry.addData("Filter YELLOW", intakeSubsystem.checkColour(IntakeSubsystem.IntakeFilter.YELLOW_ONLY));
            telemetry.addData("Filter OFF", intakeSubsystem.checkColour(IntakeSubsystem.IntakeFilter.OFF));
            telemetry.addData("Colour value", intakeSubsystem.getColourValue());
            telemetry.addData("Sample in intake, colour only", intakeSubsystem.sampleInIntakeWithColourCheck(true));
            telemetry.addData("Sample in intake", intakeSubsystem.sampleInIntakeWithColourCheck(false));
//            NormalizedRGBA rgba = hardware.colourSensor.getNormalizedColors();
//            telemetry.addData("Colour sensor as distance", hardware.colourSensor.getDistance(DistanceUnit.INCH));
//            telemetry.addData("Intake slide current", hardware.intakeSlidesM.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Pos", intakeSubsystem.slidePosition);
            telemetry.addData("Slide dis", intakeSubsystem.ticksToInchesSlidesMotor(intakeSubsystem.slidePosition));
//            telemetry.addData("Distance", intakeSubsystem.getDistance());
//            telemetry.addData("Color alpha(not normalized", hardware.colourSensor.alpha());
//            telemetry.addData("Colour", intakeSubsystem.getColorValue());
//            telemetry.addData("Colour RGBA", "R: " + rgba.red+ " G: " +rgba.green + " B: " + rgba.blue + " A: "+ rgba.alpha);
//            telemetry.addData("Pos hardware",hardware.intakeSlidesM.getCurrentPosition());
            telemetry.update();

        }
    }
}
