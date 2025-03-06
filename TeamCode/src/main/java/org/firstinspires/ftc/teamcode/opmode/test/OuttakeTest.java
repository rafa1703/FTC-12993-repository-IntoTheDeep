package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.system.accessory.math.Angles;
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
    public static OuttakeSubsystem.OuttakeClawServoState claw = OuttakeSubsystem.OuttakeClawServoState.INTAKE;
    public static double armO = OuttakeSubsystem.OuttakeArmServoState.TRANSFER_FRONT.pos, armI = IntakeSubsystem.IntakeArmServoState.TRANSFER_FRONT.pos;
    public static double wrist = OuttakeSubsystem.OuttakeWristServoState.TRANSFER_FRONT.pos, pivot = OuttakeSubsystem.OuttakePivotServoState.DOWN.pos;
    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.Red);
        hardware.FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeSubsystem = new OuttakeSubsystem(hardware);
        intakeSubsystem = new IntakeSubsystem(hardware);
        boolean cachedTurret = false;
        waitForStart();
        while (opModeIsActive())
        {
            if (!cachedTurret)
            {
                outtakeSubsystem.cacheTurretInitialPosition();
                cachedTurret = true;
            }

            hardware.resetCacheHubs();
            outtakeSubsystem.outtakeReads(true);
//            outtakeSubsystem.turretRawControl(turretPos);

            intakeSubsystem.armSetPos(armI);
            outtakeSubsystem.turretSpinTo(OuttakeSubsystem.OuttakeTurretState.TRANSFER_FRONT);
            outtakeSubsystem.wristSetPos(wrist);
            outtakeSubsystem.pivotSetPos(pivot);
            outtakeSubsystem.armSetPos(armO);
            outtakeSubsystem.clawState(claw);
//            else outtakeSubsystem.turretSpinToTicks(turretState.angle);
            telemetry.addData("Angle reducd", Angles.reduceDegrees(outtakeSubsystem.turretTicksToAngle(outtakeSubsystem.turretIncrementalPosition)));
            telemetry.addData("Integral sum", outtakeSubsystem.turretPID.returnIntegralSum());
            telemetry.addData("Turret position", outtakeSubsystem.turretIncrementalPosition);
            //telemetry.addData("Turret target ticks", outtakeSubsystem.turretAngleToTicks(turretState.angle));
            telemetry.addData("TUrret pos FL motor", hardware.FL.getCurrentPosition());
            telemetry.addData("Turret vol", outtakeSubsystem.encoderVoltage());
            telemetry.addData("Turret abs angle", outtakeSubsystem.turretAngle);
            telemetry.addData("Turret wrapped angle", Angles.normalizeDegrees(outtakeSubsystem.turretAngle));
            telemetry.addData("Turret position direct ", hardware.turretM.getCurrentPosition());
            telemetry.addData("Turret intial offset", outtakeSubsystem.initialOffsetPosition);
          //  telemetry.addData("Turret Target tick", outtakeSubsystem.turretAngleToTicks(turretPos));
//            telemetry.addData("Turret target",outtakeSubsystem.turretAngleToTicks(turretPos));
            telemetry.addData("Lift ", outtakeSubsystem.liftPosition);
            telemetry.addData("br", hardware.BR.getCurrentPosition());
            telemetry.update();
            Thread.sleep(35);
        }
    }
}
