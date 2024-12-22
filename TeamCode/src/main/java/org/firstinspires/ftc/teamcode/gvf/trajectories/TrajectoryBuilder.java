package org.firstinspires.ftc.teamcode.gvf.trajectories;


import com.arcrobotics.ftclib.util.MathUtils;

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
    private double trajectoryTimeOutTime = 1000.0; // default is 1000ms
    private double finalSpeed = 0.15; // default is 1000ms

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
    public TrajectoryBuilder addTrajectoryTimeOut(double ms)
    {
        trajectoryTimeOutTime = ms;
        return this;
    }
    /** @param speed Range 0-1**/
    public TrajectoryBuilder addFinalSpeed(double speed)
    {
        speed = MathUtils.clamp(speed, 0, 1);
        finalSpeed = speed;
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
        return new Trajectory(segments, startPose, finalPose, spatialMarkers, trajectoryTimeOutTime, finalSpeed);
    }
    //public Trajectory end() {return new Trajectory(segments, finalPose);}
}
