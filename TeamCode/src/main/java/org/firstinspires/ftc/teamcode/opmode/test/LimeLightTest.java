package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.firstinspires.ftc.teamcode.gvf.utils.Vector;
import org.firstinspires.ftc.teamcode.system.accessory.ToggleRisingEdge;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;
import org.firstinspires.ftc.teamcode.system.vision.CameraHardware;
import org.firstinspires.ftc.teamcode.system.vision.InverseKinematics;

import java.util.ArrayList;

@TeleOp(group = "Test")
public class LimeLightTest extends LinearOpMode
{
    IntakeSubsystem intakeSubsystem;
    GeneralHardware hardware;
    InverseKinematics kinematics = new InverseKinematics();
    ToggleRisingEdge toggle = new ToggleRisingEdge();
    LLResult result;
    CameraHardware cameraHardware;
    boolean foundSample;
    @Override
    public void runOpMode() throws InterruptedException
    {
        hardware = new GeneralHardware(hardwareMap, GeneralHardware.Side.RED, true);
        intakeSubsystem = new IntakeSubsystem(hardware);
        cameraHardware = new CameraHardware(hardware);
        cameraHardware.pipelineSwitch(CameraHardware.PipelineType.DETECTOR_YELLOW);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        cameraHardware.start();
        waitForStart();

        while (opModeIsActive())
        {



            if (toggle.mode(gamepad1.a))
            {
                cameraHardware.captureSnapshot("sub");
            }
//            intakeSubsystem.intakeTurret(IntakeSubsystem.IntakeTurretServoState.STRAIGHT);

            Vector sampleDis;
            result = cameraHardware.getLatestResult();
            sampleDis = InverseKinematics.distanceToSample(result.getTy(), result.getTx());
            Pose lukaPose = cameraHardware.ro2GoatMath();
            double ang = cameraHardware.sampleAngleRotatedRect(result);
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
//                telemetry.addData("Sample ang interpolation", cameraHardware.sampleAngleRatioAndInterpolation(result.getDetectorResults().get(0)));

                ArrayList<CameraHardware.Sample> samples = cameraHardware.sampleQuery(result, new Pose());
                int i=0;
                for (CameraHardware.Sample sample : samples)
                {
                    telemetry.addData("Sample " + i + ": ", sample.distanceToCrossHair);
                }
//                telemetry.addData("Sample ang ratio", cameraHardware.sampleAngleRatioAndInterpolation(result)[1]);
//                telemetry.addData("Sample ang ratio", cameraHardware.sampleAngleRatio(result));
                telemetry.addData("Sample ang rotated rect", ang);
                telemetry.addData("Sample ang", cameraHardware.sampleAngle(result));
                telemetry.addData("Sample ang 3 points", cameraHardware.sampleAngle3points(result));


            }
            if (sampleDis != null)
            {
                telemetry.addData("Distance x:", sampleDis.getX() + " y: " + sampleDis.getY());
                telemetry.addData("Distance mag", sampleDis.getMagnitude());
                telemetry.addData("Luka pose", lukaPose);
            }

            telemetry.update();
        }

    }
}
