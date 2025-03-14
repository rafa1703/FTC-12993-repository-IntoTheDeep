package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gvf.utils.Vector;
import org.firstinspires.ftc.teamcode.system.accessory.ToggleRisingEdge;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;
import org.firstinspires.ftc.teamcode.system.vision.InverseKinematics;

@TeleOp(group = "Test")
public class LimeLightTest extends LinearOpMode
{
    Limelight3A lime;
    IntakeSubsystem intakeSubsystem;
    GeneralHardware hardware;
    InverseKinematics kinematics = new InverseKinematics();
    ToggleRisingEdge toggle = new ToggleRisingEdge();
    LLResult result;
    boolean foundSample;
    @Override
    public void runOpMode() throws InterruptedException
    {
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.Red);
        intakeSubsystem = new IntakeSubsystem(hardware);
        lime = hardwareMap.get(Limelight3A.class, "limeLight");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        lime.pipelineSwitch(0);
        lime.setPollRateHz(40);

        waitForStart();
        lime.start();
        while (opModeIsActive())
        {
            LLStatus status = lime.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            if (toggle.mode(gamepad1.a))
            {
                lime.captureSnapshot("sub");
            }
            intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);

            Vector sampleDis;
            result = lime.getLatestResult();
            sampleDis = kinematics.distanceToSample(result.getTy(), result.getTx());
//            if (result != null)
//            {
//                if (result.isValid())
//                {
//                    foundSample = true;
//                    if (!gamepad1.x)
//                        intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.HORIZONTAL);
//                }
//            }
//            if (!foundSample) result = lime.getLatestResult();
//            else
//            {
//                sampleDis = kinematics.distanceToSample(result.getTy(), result.getTx());
//                intakeSubsystem.intakeSlideInternalPID(sampleDis.getY());
//                if (gamepad1.x) intakeSubsystem.intakeArm(IntakeSubsystem.IntakeArmServoState.DOWN);
//                intakeSubsystem.intakeSpin(IntakeSubsystem.IntakeSpinState.INTAKE);
//            }
            if (result != null)
            {
                telemetry.addData("Results", "Tx: " + result.getTx() + " Ty: " + result.getTy());
                telemetry.addData("Result is valid", result.isValid());
            }
            if (sampleDis != null)
            {
                telemetry.addData("Distance x:", sampleDis.getX() + " y: " + sampleDis.getY());
                telemetry.addData("Distance mag", sampleDis.getMagnitude());
            }

            telemetry.update();
        }

    }
}
