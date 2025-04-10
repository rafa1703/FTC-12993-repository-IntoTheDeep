package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

@Config
@TeleOp(name = "Outtake test", group = "Test")
public class OuttakeTest extends LinearOpMode
{
    OuttakeSubsystem outtakeSubsystem;
    IntakeSubsystem intakeSubsystem;
    GeneralHardware hardware;
    FtcDashboard dash = FtcDashboard.getInstance();
    //public static double clawPos = 0, wristPos = 0, pivotPos = 0, armPos = 0;
    //public static double intakeLefArmPos = 0, intakeRightArmPos = 0, turretPos = 0, flapPos = 0, clipPos = 0, intakeSpin = 0, intakeSlide = 0;
//    ..public static OuttakeSubsystem.OuttakeTurretState turretState = OuttakeSubsystem.OuttakeTurretState.TRANSFER_BACK;
//    public static OuttakeSubsystem.OuttakeClawServoState claw = OuttakeSubsystem.OuttakeClawServoState.INTAKE;
    public static double armO = OuttakeSubsystem.OuttakeArmServoState.INTAKE.pos, armI = IntakeSubsystem.IntakeArmServoState.HP_DEPOSIT.pos;
    public static double wrist = OuttakeSubsystem.OuttakeWristServoState.INTAKE.pos, pivot = OuttakeSubsystem.OuttakePivotServoState.RIGHT.pos;
    public static double turretI = IntakeSubsystem.IntakeTurretServoState.HP_DEPOSIT.pos;
    public static double hpReversePow = -0.15;
    public static OuttakeSubsystem.OuttakeTurretState turret = OuttakeSubsystem.OuttakeTurretState.SPEC_DEPOSIT_BACK;
//    public static OuttakeSubsystem.OuttakePivotServoState pivot = OuttakeSubsystem.OuttakePivotServoState.DOWN;
    public static double claw = 0.88, lift = 0;
    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.RED);

        outtakeSubsystem = new OuttakeSubsystem(hardware);
        intakeSubsystem = new IntakeSubsystem(hardware);
        while (!isStarted())
        {
            hardware.resetCacheHubs();
            outtakeSubsystem.outtakeReads(true);

//            telemetry.addData("Turret abs angle", outtakeSubsystem.turretAngle);
//            telemetry.addData("Turret abs voltage", outtakeSubsystem.encoderVoltage());
//            telemetry.addData("Turret angle", outtakeSubsystem.turretTicksToAngle(outtakeSubsystem.turretIncrementalPosition));
//            telemetry.addData("Turret pos", outtakeSubsystem.turretIncrementalPosition);
//            telemetry.addData("Turret pos no offset", hardware.turretM.getCurrentPosition());
//            telemetry.addData("Lift pos", outtakeSubsystem.liftPosition);
//            telemetry.addData("Turret target", turret.angle);
            telemetry.update();
        }
        waitForStart();
        while (opModeIsActive())
        {

            hardware.resetCacheHubs();
            outtakeSubsystem.outtakeReads(true);
            outtakeSubsystem.turretSpinTo(OuttakeSubsystem.OuttakeTurretState.SPEC_DEPOSIT_BACK);

//            outtakeSubsystem.pivotServoState(OuttakeSubsystem.OuttakePivotServoState.RIGHT);
            outtakeSubsystem.clawSetPos(claw);
            outtakeSubsystem.pivotSetPos(pivot);
            outtakeSubsystem.armSetPos(armO);
            outtakeSubsystem.wristSetPos(wrist);
//
//            outtakeSubsystem.wristSetPos(wrist);
//            outtakeSubsystem.liftMotorRawControl(1);
//            outtakeSubsystem.liftToInternalPID(lift);

//            intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.TRANSFER_BACK);
            if (gamepad1.a) intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
            else if (gamepad1.b) intakeSubsystem.intakeSpin(hpReversePow);
            else intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.OFF);
//            outtakeSubsystem.turretSpinToGains(turret);
//            intakeSubsystem.armSetPos(armI);
//            intakeSubsystem.intakeTurretSetPos(turretI);
//            outtakeSubsystem.turretRawControl(turretPos);
            //outtakeSubsystem.liftMotorRawControl(1);
//            intakeSubsystem.armSetPos(armI);
//            outtakeSubsystem.turretSpinTo(OuttakeSubsystem.OuttakeTurretState.TRANSFER_FRONT);
//            outtakeSubsystem.wristSetPos(wrist);
//            outtakeSubsystem.armSetPos(armO);
//            outtakeSubsystem.pivotSetPos(pivot);
//            outtakeSubsystem.clawState(claw);
//            if (gamepad1.a) intakeSubsystem.intakeSpin(0);
//            else intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
//            intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);
//            intakeSubsystem.intakeSlideInternalPID(-2);

//            outtakeSubsystem.armState(OuttakeSubsystem.OuttakeArmServoState.TRANSFER_BACK);
//            outtakeSubsystem.wristState(OuttakeSubsystem.OuttakeWristServoState.TRANSFER_BACK);
//            //outtakeSubsystem.turretRawControl(0.4);
//            outtakeSubsystem.turretSpinToGains(turret);

            telemetry.addData("Turret angle", outtakeSubsystem.turretTicksToAngle(outtakeSubsystem.turretIncrementalPosition));
            telemetry.addData("Turret pos", outtakeSubsystem.turretIncrementalPosition);
            telemetry.addData("Lift pos", outtakeSubsystem.ticksToInchesSlidesMotor(outtakeSubsystem.liftPosition));
            telemetry.addData("Turret target", turret.angle);
//            outtakeSubsystem.armSetPos(armO);
//            outtakeSubsystem.clawState(claw);
//            else outtakeSubsystem.turretSpinToTicks(turretState.angle);
//            telemetry.addData("Angle reducd", Angles.reduceDegrees(outtakeSubsystem.turretTicksToAngle(outtakeSubsystem.turretIncrementalPosition)));
//            telemetry.addData("Integral sum", outtakeSubsystem.turretPID.returnIntegralSum());
//            telemetry.addData("Turret position", outtakeSubsystem.turretIncrementalPosition);
//            //telemetry.addData("Turret target ticks", outtakeSubsystem.turretAngleToTicks(turretState.angle));
//            telemetry.addData("TUrret pos FL motor", hardware.FL.getCurrentPosition());
//            telemetry.addData("Turret vol", outtakeSubsystem.encoderVoltage());
//            telemetry.addData("Turret abs angle", outtakeSubsystem.turretAngle);
//            telemetry.addData("Turret wrapped angle", Angles.normalizeDegrees(outtakeSubsystem.turretAngle));
//            telemetry.addData("Turret position direct ", hardware.turretM.getCurrentPosition());
//            telemetry.addData("Turret intial offset", outtakeSubsystem.initialOffsetPosition);
//          //  telemetry.addData("Turret Target tick", outtakeSubsystem.turretAngleToTicks(turretPos));
////            telemetry.addData("Turret target",outtakeSubsystem.turretAngleToTicks(turretPos));
//            telemetry.addData("Lift ", outtakeSubsystem.liftPosition);
//            telemetry.addData("br", hardware.BR.getCurrentPosition());
            telemetry.update();
            Thread.sleep(35);
        }
    }
}
