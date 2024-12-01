package org.firstinspires.ftc.teamcode.gvf.trajectories;


import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.firstinspires.ftc.teamcode.gvf.utils.Vector;
import org.opencv.core.Point;

import java.util.ArrayList;

public class TrajectoryBuilder
{
    private final ArrayList<TrajectorySegment> segments = new ArrayList<>();
    private Pose finalPose = null;
    private Pose startPose = null;
    private final ArrayList<SpatialMarker> spatialMarkers = new ArrayList<>();

    public TrajectoryBuilder(Pose startPose){
        this.startPose = startPose;
    }
    public TrajectoryBuilder(){
        this.startPose = new Pose(); // we are not using the start pose for anything rn so i will make this constructor
    }

    public TrajectoryBuilder addSegment(TrajectorySegment segment){
        segments.add(segment);
        return this;
    }
    /*
    public TrajectoryBuilder addBezierSegment(Point[] segment){
        segments.add(new BezierCurveTrajectorySegment(segment));
        return this;
    }*/
    public TrajectoryBuilder addBezierSegment(Point... points){
        segments.add(new BezierCurveTrajectorySegment(points));
        return this;
    }
    public TrajectoryBuilder addBezierSegment(double maxSpeed, Point... points){
        segments.add(new BezierCurveTrajectorySegment(maxSpeed, points));
        return this;
    }
    public TrajectoryBuilder addFinalPose(Pose pose)
    {
        finalPose = pose;
        return this;
    }
    public TrajectoryBuilder addFinalPose(double x, double y, double heading)
    {
        finalPose = new Pose(x, y, heading);
        return this;
    }
    public TrajectoryBuilder addSpatialMarker(Pose markerPose, Callback callback)
    {
        spatialMarkers.add(new SpatialMarker(markerPose, callback));
        return this;
    }

    public Trajectory build(){
        if (finalPose == null) // this is use when following tangentially that we dont know what will be the tangent heading
        {
            Vector finalVector = segments.get(segments.size() -1).getEndPoint(); // transform vector into the final pose
            finalPose = new Pose(finalVector.getX(), finalVector.getY(), finalVector.getAngle()); // keep in mind this angle is wrong as it gets the angle from arctan(y/x).
        }
        return new Trajectory(segments, startPose, finalPose, spatialMarkers);
    }
    //public Trajectory end() {return new Trajectory(segments, finalPose);}
}
