package org.firstinspires.ftc.teamcode.system.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

public class CameraHardware
{
    private Limelight3A limelight;
    private LLResult latestResult = null;
    public CameraHardware(GeneralHardware hardware)
    {
        this.limelight = hardware.limelight;
        hardwareSetUp(hardware);
    }

    public void hardwareSetUp(GeneralHardware hardware)
    {
        limelight.setPollRateHz(50); // This sets how often we ask Limelight for data (50 times per second)
        if (hardware.side == GeneralHardware.Side.Blue)
            limelight.pipelineSwitch(0);
        else  limelight.pipelineSwitch(1);
    }
    public void deleteSnapshot()
    {
        limelight.deleteSnapshots();
    }
    public void captureSnapshot(String name)
    {
        limelight.captureSnapshot(name);
    }
    public LLResult getLatestResult()
    {
        latestResult = limelight.getLatestResult();
        return latestResult;
    }
    public boolean isResultValid()
    {
        return latestResult.isValid();
    }
    public boolean isDataFresh()
    {
        long staleness = latestResult.getStaleness();
        return staleness < 50;
    }
    public void start()
    {
        limelight.start();
    }
    public void close()
    {
        limelight.close();
    }
    public void pause()
    {
        limelight.pause();
    }
}
