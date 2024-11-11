package org.firstinspires.ftc.teamcode.gvf.trajectories;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.firstinspires.ftc.teamcode.gvf.utils.Vector;
import org.opencv.core.Point;

import java.util.ArrayList;

public class Trajectory
{
    private ArrayList<TrajectorySegment> segments;
    private int numberOfSegments;
    GVFLogic gvfLogic = new GVFLogic();
    ArrayList<Point> fullCurve;
    private double threshold = 1.5; // this is the default
    private boolean isFinished = false;
    private boolean usePID = false;
    private Pose startPose, finalPose;

    private ArrayList<SpatialMarker> spatialMarkers;
    private int prevU;
    private double prevT;
    private double timer = 0;
    private double HIGHER_DIMENSION_THRESHOLD;

    public Trajectory(TrajectorySegment segment)
    {
        segments.add(segment);
        init();
    }
    public Trajectory(ArrayList<TrajectorySegment> segments)
    {
        this.segments = segments;
        init();
    }

    public Trajectory(ArrayList<TrajectorySegment> segments, @NonNull Pose startPose, @NonNull Pose finalPose, ArrayList<SpatialMarker> spatialMarkers)
    {
        this(segments, startPose, finalPose, spatialMarkers, 500.0); // 500ms is the default
    }
    public Trajectory(ArrayList<TrajectorySegment> segments, @NonNull Pose startPose, @NonNull Pose finalPose, ArrayList<SpatialMarker> spatialMarkers, double timerThreshold)
    {
        this.segments = segments;
        this.startPose = startPose;
        this.finalPose = finalPose;
        this.spatialMarkers = spatialMarkers;
        this.HIGHER_DIMENSION_THRESHOLD = timerThreshold;
        init();
    }


    public void init()
    {
        numberOfSegments = segments.size() - 1;
        fullCurve = returnFullCurve();
        for (SpatialMarker marker : spatialMarkers)
        {
            double closestDistance = Double.POSITIVE_INFINITY;
            for (TrajectorySegment segment : segments)
            {
                Point distAndT = segment.getClosestDistanceAndT(marker.spatialPoint.toPoint());
                if (distAndT.y < closestDistance)
                {
                    closestDistance = distAndT.y;
                    marker.t = distAndT.x;
                    marker.u = segments.indexOf(segment);
                }
            }
        }
    }

    public Vector getPowerVector(Pose pose) // this should work lol, idk fucking know tho
    {
        gvfLogic.setReverse(false);
        gvfLogic.setFollowTangentially(false);
        gvfLogic.setSplineHeading(false);

        double closestDistance = Double.POSITIVE_INFINITY;
        double t = 0;
        int u = 0;
        if (prevU == numberOfSegments && prevT == 1) //if we were following the last point in the curve no need to draw cpu power
        {
            t = prevT;
            u = prevU;
            if (timer == 0)
            {
                timer = System.currentTimeMillis();
            }
            if (Math.abs(System.currentTimeMillis() - timer) > HIGHER_DIMENSION_THRESHOLD)
            {
                isFinished = true;
            }
        }
        else
        {
            for (TrajectorySegment segment : segments)
            {
                Point distAndT = segment.getClosestDistanceAndT(pose.toPoint());
                if (distAndT.y < closestDistance)
                {
                    closestDistance = distAndT.y;
                    t = distAndT.x;
                    u = segments.indexOf(segment);
                }
            }
        }
        //TODO: add an exit condition so we dont calculate if we finished
        BezierCurve curve = segments.get(u).returnCurve();
        boolean lastCurve = u == numberOfSegments;

        // passing t here removes another full iteration of the curve
        Vector powerVector = gvfLogic.calculate(curve, t, pose, lastCurve, segments.get(u).getMaxSpeed());

        if(!spatialMarkers.isEmpty()) // hope this doesn't break shit
        {
            for(SpatialMarker marker : spatialMarkers)
            {
                if(t >= marker.t && u >= marker.u) // this guarantee that the spatial marker will run on the closest point
                {
                    marker.callback.onMarker();
                    spatialMarkers.remove(marker);
                }
            }
        }

        // if we less then the threshold we can say we are finished
        if (segments.get(numberOfSegments).getEndPoint().subtract(pose.toPoint()).getMagnitude() < threshold)
        {
            isFinished = true;
        }

        usePID = gvfLogic.usePID() && lastCurve && finalPose != null;
        prevU = u;
        prevT = t;

        return powerVector;
    }
    public Vector getTangentPowerVector(Pose pose, boolean reverse) // this should work lol, idk fucking know tho
    {
        gvfLogic.setReverse(reverse);
        gvfLogic.setFollowTangentially(true);
        gvfLogic.setSplineHeading(false);

        double closestDistance = Double.POSITIVE_INFINITY;
        double t = 0;
        int u = 0;
        if (prevU == numberOfSegments && prevT == 1) //if we were following the last point in the curve no need to draw cpu power
        {
            t = prevT;
            u = prevU;
            if (timer == 0)
            {
                timer = System.currentTimeMillis();
            }
            if (Math.abs(System.currentTimeMillis() - timer) > HIGHER_DIMENSION_THRESHOLD)
            {
                isFinished = true;
            }
        }
        else
        {
            for (TrajectorySegment segment : segments)
            {
                Point distAndT = segment.getClosestDistanceAndT(pose.toPoint());
                if (distAndT.y <= closestDistance)
                {
                    closestDistance = distAndT.y;
                    t = distAndT.x;
                    u = segments.indexOf(segment);
                }
            }
        }
        BezierCurve curve;
        boolean lastCurve = u == numberOfSegments;
        curve = segments.get(u).returnCurve();

        Vector powerVector = gvfLogic.calculate(curve, t, pose, lastCurve, segments.get(u).getMaxSpeed());
        // if we less then the threshold we can say we are finished
        if (segments.get(numberOfSegments).getEndPoint().subtract(pose.toPoint()).getMagnitude() < threshold)
        {
            isFinished = true;
        }
        if(!spatialMarkers.isEmpty()) // hope this doesn't break shit
        {
            for(SpatialMarker marker : spatialMarkers)
            {
                if(t >= marker.t && u >= marker.u) // this guarantee that the spatial marker will run on the closest point
                {
                    marker.callback.onMarker();
                    spatialMarkers.remove(marker);
                }
            }
        }
        usePID = gvfLogic.usePID() && lastCurve && finalPose != null;
        prevU = u;
        prevT = t;

        return powerVector;
    }
    public Vector getPowerVectorSplineHeading(Pose pose) // this should work lol, idk fucking know tho
    {
        gvfLogic.setReverse(false);
        gvfLogic.setFollowTangentially(false);
        gvfLogic.setSplineHeading(true);

        double closestDistance = Double.POSITIVE_INFINITY;
        double t = 0;
        int u = 0;
        if (prevU == numberOfSegments && prevT == 1) //if we were following the last point in the curve no need to draw cpu power
        {
            t = prevT;
            u = prevU;
            if (timer == 0)
            {
                timer = System.currentTimeMillis();
            }
            if (Math.abs(System.currentTimeMillis() - timer) > HIGHER_DIMENSION_THRESHOLD)
            {
                isFinished = true;
            }
        }
        else
        {
            for (TrajectorySegment segment : segments)
            {
                Point distAndT = segment.getClosestDistanceAndT(pose.toPoint());
                if (distAndT.y < closestDistance)
                {
                    closestDistance = distAndT.y;
                    t = distAndT.x;
                    u = segments.indexOf(segment);
                }
            }
        }
        BezierCurve curve = segments.get(u).returnCurve();
        boolean lastCurve = u == numberOfSegments;


        Vector powerVector = gvfLogic.calculate(curve, t, pose, lastCurve, segments.get(u).getMaxSpeed());
        // we need to change the z component of the power vector, yes this being done here is dodgy as fuck but i don't care
        // (it needs to be here because i want to interpolate using the u value of the trajectory and not the t value
        double finalHeading = finalPose.getHeading();
        double headingDiff = gvfLogic.headingInterpolation(pose.getHeading(), finalHeading, (u + t) / (numberOfSegments + 1)) - pose.getHeading(); // remove heading because we want the diff
        //headingDiff = normalizeRadians(headingDiff - pose.getHeading());
        powerVector = new Vector(powerVector.getX(), powerVector.getY(), headingDiff);

        // if we less then the threshold we can say we are finished
        if (segments.get(numberOfSegments).getEndPoint().subtract(pose.toPoint()).getMagnitude() < threshold)
        {
            isFinished = true;
        }

        if(!spatialMarkers.isEmpty()) // hope this doesn't break shit
        {
            for(SpatialMarker marker : spatialMarkers)
            {
                if(t >= marker.t && u >= marker.u) // this guarantee that the spatial marker will run on the closest point
                {
                    marker.callback.onMarker();
                    spatialMarkers.remove(marker);
                }
            }
        }
        usePID = gvfLogic.usePID() && lastCurve && finalPose != null;
        prevU = u;
        prevT = t;
        return powerVector;
    }
    private ArrayList<Point> returnFullCurve()
    {
        ArrayList<Point> wholeCurve = new ArrayList<>();
        for (TrajectorySegment segment : segments)
        {
            ArrayList<Point> curve = segment.returnCurve().returnCurve();
            wholeCurve.addAll(curve);
        }
        return wholeCurve;
    }

    public boolean isFinished()
    {
        return isFinished;
    }

    public ArrayList<Point> getFullCurve()
    {
        return fullCurve;
    }

    public void setThreshold(double threshold)
    {
        this.threshold = threshold;
    }

    public boolean usePid()
    {
        return usePID;
    }

    public Pose getFinalPose()
    {
        return finalPose;
    }
    public double length()
    {
        // this returns the sampled number of the path
        return (fullCurve.size() + 1) * segments.get(0).returnCurve().getInterval();
    }
    public double getTangentFinalHeading()
    {
        return gvfLogic.getTangentFinalHeading();
    }
}
