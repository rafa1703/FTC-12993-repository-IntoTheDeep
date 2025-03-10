package org.firstinspires.ftc.teamcode.system.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.List;

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
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (50 times per second)
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
    public void updatePythonInputs(double[] inputs)
    {
        limelight.updatePythonInputs(inputs);
    }
    public double sampleAngle()
    {
        List<List<Double>> corners = latestResult.getColorResults().get(0).getTargetCorners();
        List<Point> cornerPoints = new ArrayList<>();
        for (List<Double> point : corners)
        {
            cornerPoints.add(new Point(point.get(0), point.get(1)));
        }

        double maxDistance = Double.NEGATIVE_INFINITY;
        Point a = new Point();
        Point b = new Point();
        for (int i = 0; i < cornerPoints.size(); i++)
        {
            Point firstPoint = cornerPoints.get(i);
            Point secondPoint = cornerPoints.get(i + 1);
            double distance = Math.sqrt(Math.pow(secondPoint.x - firstPoint.x, 2) + Math.pow(secondPoint.y - firstPoint.y, 2));
            if (distance > maxDistance)
            {
                maxDistance = distance;
                a = firstPoint;
                b = secondPoint;
            }
        }
        double slope = (b.y - a.y) / (b.x - a.x);
        return Math.atan(slope);
    }
    public double sampleAngle3points()
    {
        List<List<Double>> corners = latestResult.getColorResults().get(0).getTargetCorners();
        List<Point> cornerPoints = new ArrayList<>();
        for (List<Double> point : corners)
        {
            cornerPoints.add(new Point(point.get(0), point.get(1)));
        }
        Point a = cornerPoints.get(0);
        Point b = cornerPoints.get(3);
        Point c = cornerPoints.get(2);
        double m1 = (a.y - b.y) / (a.x - b.x); // slope
        double m2 = (c.y - b.y) / (c.x - b.x);
        // tan theta = abs((m1- m2) / 1 + m1*m2)
        double angle = Math.atan((m1 - m2) / (1 + m1 * m2));
        if (angle < 0) angle += Math.PI / 2; // same as running the absolute/
        return Math.toDegrees(angle);

    }

}
