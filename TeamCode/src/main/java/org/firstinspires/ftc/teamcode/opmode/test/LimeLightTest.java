package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gvf.utils.Vector;
import org.firstinspires.ftc.teamcode.system.accessory.ToggleRisingEdge;
import org.firstinspires.ftc.teamcode.system.vision.InverseKinematics;

@TeleOp(group = "Test")
public class LimeLightTest extends LinearOpMode
{
    Limelight3A lime;
    InverseKinematics kinematics = new InverseKinematics();
    ToggleRisingEdge toggle = new ToggleRisingEdge();
    @Override
    public void runOpMode() throws InterruptedException
    {
        lime = hardwareMap.get(Limelight3A.class, "limeLight");

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
            LLResult result = lime.getLatestResult();
            Vector sampleDis = kinematics.distanceToSample(result.getTy(), result.getTx());
            telemetry.addData("Results", "Tx: " + result.getTx() + " Ty: " + result.getTy());
            telemetry.addData("Distance x:", sampleDis.getX() + " y: " + sampleDis.getY());
            telemetry.addData("Distance mag", sampleDis.getMagnitude());
            telemetry.addData("Result is valid", result.isValid());
            telemetry.update();
        }

    }
}
