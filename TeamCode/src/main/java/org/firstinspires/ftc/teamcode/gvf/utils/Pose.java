package org.firstinspires.ftc.teamcode.gvf.utils;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.opencv.core.Point;

public class Pose
{
    private final double x, y, heading;
    private final double tolerance;
    public static double defaultTolerance = 1; // half an inch that is rr default

    public Pose(double x, double y, double heading, double tolerance)
    {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.tolerance = tolerance;
    }

    public Pose(double x, double y, double heading)
    {
        this(x, y, heading, defaultTolerance);
    }

    public Pose(double x, double y)
    {
        this(x, y, 0, defaultTolerance);
    }

    public Pose()
    {
        this(0, 0, 0, defaultTolerance);
    }
    public Pose(Pose pose)
    {
        this(pose.getX(), pose.getY(), pose.getHeading(), pose.tolerance);
    }

    public Pose(Pose2d pose2d)
    {
        this(pose2d.getX(), pose2d.getY(), pose2d.getHeading());
    }
    public Pose(Pose2D pose2D)
    {
        this(pose2D.getX(DistanceUnit.INCH), pose2D.getY(DistanceUnit.INCH), pose2D.getHeading(AngleUnit.RADIANS));
    }

    public Pose(SparkFunOTOS.Pose2D pose2D)
    {
        this(pose2D.x, pose2D.y, pose2D.h);
    }

    public double getX(){
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }

    public double getTolerance(){
        return tolerance;
    }

    public double getDistance(Pose other){
        return Math.sqrt((other.getX() - x)*(other.getX() - x) + (other.getY() - y)*(other.getY() - y));
    }

    public boolean isReached(Pose other){
        return getDistance(other) <= tolerance;
    }

    public Pose plus(Pose other){
        return new Pose(x + other.x, y + other.y, heading + other.heading);
    }

    public Pose2d toPose2d(){
        return new Pose2d(x,y,heading);
    }
    public Pose2D toPose2D()
    {
        return new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, heading);
    }

    public Point toPoint()
    {
        return new Point(x, y);
    }
    @NonNull
    @Override
    public String toString(){
        return String.valueOf(x) + " " + String.valueOf(y) + " " + String.valueOf(heading);
    }

}