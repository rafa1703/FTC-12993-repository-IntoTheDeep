package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class LimeLightTest extends LinearOpMode
{
    Limelight3A lime;
    @Override
    public void runOpMode() throws InterruptedException
    {
        lime = hardwareMap.get(Limelight3A.class, "lime");

        lime.pipelineSwitch(0);

        waitForStart();

        while (opModeIsActive())
        {
            LLStatus status = lime.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = lime.getLatestResult();
        }

    }
}
