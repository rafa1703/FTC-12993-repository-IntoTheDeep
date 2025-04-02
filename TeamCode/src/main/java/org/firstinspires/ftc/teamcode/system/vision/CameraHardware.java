package org.firstinspires.ftc.teamcode.system.vision;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.firstinspires.ftc.teamcode.gvf.utils.Vector;
import org.firstinspires.ftc.teamcode.system.hardware.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;
import org.opencv.core.Core;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class CameraHardware
{
    public enum PipelineType
    {
        RED(0),
        BLUE(1),
        YELLOW(3),
        RED_YELLOW(2),
        BLUE_YELLOW(4),
        DETECTOR_YELLOW(7);

        public final int pipelineIndex;
        PipelineType(int pipelineIndex)
        {
            this.pipelineIndex = pipelineIndex;
        }
    }
    private Limelight3A limelight;
    private LLResult latestResult = null;
    private InterpLUT interpLUT = new InterpLUT();
    private double lowerBound, upperBound;
    public CameraHardware(GeneralHardware hardware)
    {
        this.limelight = hardware.limelight;
        hardwareSetUp(hardware);
    }
    public CameraHardware(GeneralHardware hardware, int pipelineIndex)
    {
        this.limelight = hardware.limelight;
        hardwareSetUp(hardware);
        limelight.pipelineSwitch(pipelineIndex);
    }

    public void hardwareSetUp(GeneralHardware hardware)
    {
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (50 times per second)
        if (hardware.side == GeneralHardware.Side.RED)
            limelight.pipelineSwitch(0);
        else  limelight.pipelineSwitch(1);

        lowerBound = 0.77;
        upperBound = 1.8;
        interpLUT.add(lowerBound, 90);
        interpLUT.add((lowerBound + upperBound) / 2, 45);
        interpLUT.add(upperBound, 0);
        interpLUT.createLUT();
    }
    public void pipelineSwitch(PipelineType type)
    {
        limelight.pipelineSwitch(type.pipelineIndex);
    }
    public void deleteSnapshot()
    {
        limelight.deleteSnapshots();
    }
    public boolean captureSnapshot(String name)
    {
        return limelight.captureSnapshot(name);
    }
    //        limelight.getLatestResult().getDetectorResults().get().getClassName()
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
    public static double calculateDistanceOY(double targetHeight, double cameraHeight, double ty, double mountingAngle) {

        double totalAngleRadians = Math.toRadians(ty + mountingAngle);


        return ((Math.abs(targetHeight - cameraHeight)) *(Math.abs(1 / Math.tan(totalAngleRadians))));
    }
//    public void ro2GoatMath(Limelight3A limelight)
//    {
//        LLResult result = limelight.getLatestResult();
//        double i = 0;
//        double extendomin = 999999;
//        double anglebun = -1;
//
//        double targetHeight = 1.5;
//        double cameraHeight = 9.84;
//        double mountingAngle = -15;
//
//        try{
//            if(result.isValid()) {
//                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
//                for (LLResultTypes.ColorResult colorResult : colorResults) {
//
//                    double ty = colorResult.getTargetYDegreesNoCrosshair();
//                    double tx = colorResult.getTargetXDegreesNoCrosshair();
//
//                    double distanceOY = calculateDistanceOY(targetHeight, cameraHeight, ty, mountingAngle, 0);
//                    double distanceOX = distanceOY * Math.abs(Math.tan(Math.toRadians(tx)));
//
//                    double x= Math.atan(distanceOX / distanceOY) * Math.signum(ty); // *Math.signum(tx) - Math.toRadians(5)
//                    double poz = calculateExtendo(Math.sqrt(distanceOX * distanceOX + distanceOY * distanceOY) -12);
//
//                    // if((poz <= extendomin || i == 0) && poz < 40000)
//                    // {
//                    extendomin = poz;
//                    anglebun = x;
//                    //  }
//
//
//                    globals.angleturn = anglebun;
//                    globals.extedo_limelight_pos = Math.max(extendomin, 9500);
//                    globals.didthelimelightdetectcorrectly = true;
//                    i++;
//
//    //                if (7 < distanceOX && distanceOX < 11 && tx < 0) {
//    //                    globals.allign_yellow_submerssible_limelight = globals.auto_sample_sumbersiblex[0] + (distanceOX - 17) / 2.54;
//    //                    globals.extedo_limelight_pos = Math.max(calculateExtendo( distanceOY - y * 2.54), calculateExtendo(7));
//    //                    globals.didthelimelightdetectcorrectly = true;
//    //                   // limelight.shutdown();
//    //                }
//                    //limelight.shutdown();
//                }
//
//            }
//
//
//        } catch (Exception e) {}
//    }
    public Pose ro2GoatMath(LLResultTypes.DetectorResult result)
    {
        double targetHeight = 1.5;
        double cameraHeight = 9.84;
        double mountingAngle = -9.35;

        try{
            if(result != null) {
                double ty = result.getTargetXDegreesNoCrosshair();
                double tx = result.getTargetYDegreesNoCrosshair();

                double distanceOY = calculateDistanceOY(targetHeight, cameraHeight, ty, mountingAngle);
                double distanceOX = distanceOY * Math.abs(Math.tan(Math.toRadians(tx)));

                double h = Math.atan(distanceOX / distanceOY) * Math.signum(ty); // *Math.signum(tx) - Math.toRadians(5)
                return new Pose(distanceOX * Math.signum(tx), distanceOY, 0);
            }
            return null;
        } catch (Exception e) {
            return null;
        }
    }
    public Pose ro2GoatMath()
    {
        return ro2GoatMath(limelight);
    }
    public Pose ro2GoatMath(Limelight3A limelight)
    {
        LLResult result = limelight.getLatestResult();

        double targetHeight = 1.5;
        double cameraHeight = 9.84;
        double mountingAngle = -9.35;

        try{
            if(result.isValid()) {
                double ty = result.getTyNC();
                double tx = result.getTxNC();

                double distanceOY = calculateDistanceOY(targetHeight, cameraHeight, ty, mountingAngle);
                double distanceOX = distanceOY * Math.abs(Math.tan(Math.toRadians(tx)));

                double h = Math.atan(distanceOX / distanceOY) * Math.signum(ty); // *Math.signum(tx) - Math.toRadians(5)
                return new Pose(distanceOX * Math.signum(tx), distanceOY, h);
            }
            return null;
        } catch (Exception e) {
            return null;
        }
    }
    public double sampleAngle(LLResult result)
    {
        if (result.getColorResults().isEmpty()) return Double.NaN;
        List<List<Double>> corners = result.getColorResults().get(0).getTargetCorners();
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
            Point secondPoint = cornerPoints.get((i + 1) % 4);
            double distance = Math.sqrt(Math.pow(secondPoint.x - firstPoint.x, 2) + Math.pow(secondPoint.y - firstPoint.y, 2));
            if (distance > maxDistance)
            {
                maxDistance = distance;
                a = firstPoint;
                b = secondPoint;
            }
        }
        double slope = (b.y - a.y) / (b.x - a.x);
        return Math.toDegrees(Math.atan(slope));
    }
    public double sampleAngleRatio(LLResult result)
    {
        if (result.getDetectorResults().isEmpty()) return Double.NaN;
        List<List<Double>> corners = result.getDetectorResults().get(0).getTargetCorners();
        Point[] cornerPoints = {
                new Point(corners.get(0).get(0), corners.get(0).get(1)),
                new Point(corners.get(1).get(0), corners.get(1).get(1)),
                new Point(corners.get(2).get(0), corners.get(2).get(1)),
                new Point(corners.get(3).get(0), corners.get(3).get(1))
        };

        MatOfPoint2f matOfPoint = new MatOfPoint2f();
        matOfPoint.fromArray(cornerPoints);
        RotatedRect rotatedRect = Imgproc.minAreaRect(matOfPoint);
        Rect boundingRect = rotatedRect.boundingRect();
        return (1 - (((double) boundingRect.width / boundingRect.height)  / (3.5 / 1.5))) * 90; // ratio 2.33
//        return rotatedRect.angle;
    }
    public double sampleAngleRatioAndInterpolation(LLResultTypes.DetectorResult result)
    {
        if (result == null) return Double.NaN;
        List<List<Double>> corners = result.getTargetCorners();
        Point[] cornerPoints = {
                new Point(corners.get(0).get(0), corners.get(0).get(1)),
                new Point(corners.get(1).get(0), corners.get(1).get(1)),
                new Point(corners.get(2).get(0), corners.get(2).get(1)),
                new Point(corners.get(3).get(0), corners.get(3).get(1))
        };

        MatOfPoint2f matOfPoint = new MatOfPoint2f();
        matOfPoint.fromArray(cornerPoints);
        RotatedRect rotatedRect = Imgproc.minAreaRect(matOfPoint);
        Rect boundingRect = rotatedRect.boundingRect();
        double ratio = (double) boundingRect.width / boundingRect.height;
        if (ratio < lowerBound) return 90;// interpLUT.get(lowerBound + 0.01);
        if (ratio > upperBound) return 0; //interpLUT.get(upperBound - 0.01);
        return interpLUT.get(ratio);
    }
    private double interpolation(double p1, double p2, double t)
    {
        return (1 - t) * p1 + t * p2;
    }
    public double sampleAngleRotatedRect(LLResult result)
    {
        if (result.getDetectorResults().isEmpty()) return Double.NaN;
        MatOfPoint2f matOfPoint = getMatOfPoint2f(result);
        RotatedRect rotatedRect = Imgproc.minAreaRect(matOfPoint);
        if ((double) rotatedRect.boundingRect().width / rotatedRect.boundingRect().height < 1.3) return 0;
        if (rotatedRect.boundingRect().width < rotatedRect.boundingRect().height) return rotatedRect.angle + 180;
        else return rotatedRect.angle + 90;
//        return rotatedRect.angle;
    }

    @NonNull
    private static MatOfPoint2f getMatOfPoint2f(LLResult result)
    {
        List<List<Double>> corners = result.getDetectorResults().get(0).getTargetCorners();
        Point[] cornerPoints = {
                new Point(corners.get(0).get(0), corners.get(0).get(1)),
                new Point(corners.get(1).get(0), corners.get(1).get(1)),
                new Point(corners.get(2).get(0), corners.get(2).get(1)),
                new Point(corners.get(3).get(0), corners.get(3).get(1))
        };

        MatOfPoint2f matOfPoint = new MatOfPoint2f();
        matOfPoint.fromArray(cornerPoints);
        return matOfPoint;
    }
    public ArrayList<Sample> sampleQuery(LLResult result, Pose robot)
    {
        if (result.getDetectorResults().isEmpty()) return null;
        ArrayList<Sample> sampleList = new ArrayList<>();

        for (LLResultTypes.DetectorResult dr : result.getDetectorResults())
        {
            double ang = sampleAngleRatioAndInterpolation(dr);
            Pose robotToSample = ro2GoatMath(dr);
            Vector s = new Vector(robotToSample.getX(), robotToSample.getY());

            sampleList.add(new Sample(sampleColourByID(dr.getClassId()), ang, s, robotToSample.plus(robot),Math.abs(dr.getTargetXDegrees()) + Math.abs(dr.getTargetYDegrees()), dr)); // we store the abs angle to the crosshair, smaller value is the closer to the ideal intake point
            sampleList.sort(Comparator.comparingDouble(s2 -> s2.detectorResult.getTargetYDegrees()));
        }
        // sort samples ?
        return sampleList;
    }

    //(s1, s2) -> Double.compare(s1.distanceToCrossHair, s2.distanceToCrossHair)
    Comparator<Sample> sampleComparator = (sample, t1) -> Double.compare(sample.distanceToCrossHair, t1.distanceToCrossHair);
    private double disBetweenTwoPoints(Point a, Point b)
    {
        return Math.sqrt(Math.pow(b.x - a.x, 2) + Math.pow(b.y - a.y, 2));
    }
    private IntakeSubsystem.SampleColour sampleColourByID(int id)
    {
        switch (id)
        {
            case 0:
                return IntakeSubsystem.SampleColour.RED;
            case 1:
                return IntakeSubsystem.SampleColour.BLUE;
            default:
                return IntakeSubsystem.SampleColour.YELLOW;
        }
    }
//    public double sampleAngleCvRect(LLResult result)
//    {
//        if (result.getColorResults().isEmpty()) return Double.NaN;
//        List<List<Double>> corners = result.getColorResults().get(0).getTargetCorners();
//        MatOfPoint2f
//        RotatedRect rotatedRect = new RotatedRect();
//    }
    public double sampleAngle3points(LLResult result)
    {
        if (result.getColorResults().isEmpty()) return Double.NaN;

        List<List<Double>> corners = result.getColorResults().get(0).getTargetCorners();
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
    public class Sample
    {
        public IntakeSubsystem.SampleColour colour;
        public double angle;
        public Vector vectorToDetectPose;
        public Pose pose;
        public LLResultTypes.DetectorResult detectorResult;
        public double distanceToCrossHair;

        public Sample(IntakeSubsystem.SampleColour colour, double angle, Vector vectorToDetectPose, Pose pose, double distanceToCrossHair, LLResultTypes.DetectorResult detectorResult)
        {
            this.colour = colour;
            this.angle = angle;
            this.vectorToDetectPose = vectorToDetectPose;
            this.pose = pose;
            this.distanceToCrossHair = distanceToCrossHair;
            this.detectorResult = detectorResult;
        }

        @NonNull
        @Override
        public String toString()
        {
            return "Colour: " + colour + " Angle: " + angle + " Pose: " + " Vector: " + vectorToDetectPose.toString()
                    + " TyNc: " + detectorResult.getTargetXDegreesNoCrosshair() + " TxNc: " + detectorResult.getTargetXDegreesNoCrosshair();
        }
    }
}


