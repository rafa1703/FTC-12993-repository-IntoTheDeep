package org.firstinspires.ftc.teamcode.gvf.trajectories;

import static org.firstinspires.ftc.teamcode.system.hardware.Globals.normalizeRadians;

import static java.lang.Math.PI;

import androidx.annotation.NonNull;


import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.firstinspires.ftc.teamcode.gvf.utils.Vector;
import org.opencv.core.Point;

public class GVFLogic
{

    //This whole
    public boolean followTangentially = false;
    public boolean reverse = false;
    public boolean splineHeading = false;
    public boolean usePID = false;
    public double tangentFinalHeading;
    public double centripetalCorrectionCoefficient = 0.006;
    public Vector calculate(@NonNull BezierCurve curve, double t, Pose pose, boolean slowDown,double slowSpeed, double maxSpeed)
    {
        Point robot = pose.toPoint();
        Vector endPoint = curve.getEndPoint();
        //double t = curve.returnClosestT(robot);
        Vector closestPoint = new Vector(curve.parametric(t));
        Vector derivative = curve.getTangentialVector(t);
        Vector robotToPoint = closestPoint.subtract(robot);
        Vector robotToEnd = endPoint.subtract(robot);
        // TODO i think if i tune this better i can run a normal predictive thing
        double CORRECTION_DIS = 12; // this should probably be closer to half a tile
        double SAVING_THROW_DIS = 5; // this should be way less
        double SLOWDOWN_DIS = 34; // this maybe has to be interpolated based on how much to slow down

        double directPursuitThreshold = 1;
        // going from above now should not break anything and improve cycle times
        for(double i = 1; i >= 0; i -= 1 / curve.getInterval()) // okay so this is like at the point lets iterate trough all points and find out if the distance of the point to the end point if its lower than the threshold
        {
            double dist = endPoint.subtract(curve.parametric(i)).getMagnitude();
            // if we are closer than the threshold we can start directly following the curve
            if (dist < SAVING_THROW_DIS)
            {
                directPursuitThreshold = i;
                break;
            }
        }

        double correctionScale = Math.min(1, robotToPoint.getMagnitude() / CORRECTION_DIS);
        double direction = headingInterpolation(derivative.getAngle(), robotToPoint.getAngle(), correctionScale);

        Vector tempRobot = new Vector(robot.x, robot.y);
        // this just say if the closest point and the robot is tangent or t is greater than the follow the path threshold we follow the end point
        if ((t == 1 && Math.abs(tempRobot.subtract(closestPoint).getAngle() - derivative.getAngle()) <= 0.5 * PI)
                || t >= directPursuitThreshold)
        {
            direction = endPoint.subtract(robot).getAngle();
        }

        Vector movementVector = new Vector(Math.cos(direction), Math.sin(direction));
        double speed = maxSpeed;

        if (robotToEnd.getMagnitude() < SLOWDOWN_DIS && slowDown)
        {
            // like a weighted average for the speed
            speed = interpolation(slowSpeed, speed, robotToEnd.getMagnitude() / SLOWDOWN_DIS);
        }
        movementVector.scaleBy(speed);

        if (followTangentially && !splineHeading) // reverse only affects if we following tangentially so it can be nested here
        {
            if(t == 1 && slowDown) // we only slowdown on the last curve
            {
                derivative = curve.getTangentialVector(1 - (1/ curve.getInterval())); // this is because at 1 the derivative is NaN
            }
            double angle = derivative.getAngle();
            if (reverse) angle += Math.toRadians(180);
            if(t==1 && slowDown) tangentFinalHeading = angle;
            double headingScale = Math.abs(Math.min(normalizeRadians(angle - pose.getHeading()) / Math.toRadians(50), 1)); // rn i think 70 is the best angle to consider max thing
            double headingDiff = headingInterpolation(pose.getHeading(), angle, 1) - pose.getHeading();  //Math.min(normalizeRadians(derivative.getAngle() - pose.getHeading()) / Math.toRadians(30), 1); // Who the fuck knows if this is gonna work
            // we want to remove the current heading because interpolation at t = 0 returns the current heading
            movementVector = new Vector(movementVector.getX(), movementVector.getY(), headingDiff);

        }
        // tbh idk if the dist check is necessary here
        usePID = t == 1 && robotToEnd.getMagnitude() < 4;
        return movementVector;

    }

    public double headingInterpolation(double theta1, double theta2, double t)
    {
        //Normalize radians with a scaled (t)
        double diff = theta2 - theta1;
        diff %= 2 * PI;
        if (Math.abs(diff) > PI) {
            if (diff > 0) {
                diff -= 2 * PI;
            } else {
                diff += 2 * PI;
            }
        }
        return theta1 + t * diff;
    }

    private double interpolation(double p1, double p2, double t) {
        return (1 - t) * p1 + t * p2;
    }

    public void setFollowTangentially(boolean followTangentially)
    {
        this.followTangentially = followTangentially;
    }

    public void setSplineHeading(boolean splineHeading)
    {
        this.splineHeading = splineHeading;
    }

    public void setReverse(boolean reverse)
    {
        this.reverse = reverse;
    }

    public boolean isFollowTangentially()
    {
        return followTangentially;
    }

    public boolean isReverse()
    {
        return reverse;
    }

    public boolean usePID()
    {
        return usePID;
    }
    public double getTangentFinalHeading()
    {
        return tangentFinalHeading;
    }

    public double centripetalForce(double r, Vector velocityVector)
    {
        double m = 0;
        double v = velocityVector.getMagnitude();
        return m * (v * v) / r;
    }

    public void centripetalCorrectionVector(BezierCurve curve , double t, double velocity)
    {

        Vector velocityVectorAtT = curve.getTangentialVector(t);
        double theta = velocityVectorAtT.getAngle() + PI/2.0;

        Vector centripetalCorrectionVector = new Vector(Math.cos(theta), Math.sin(theta))
                .scaledBy(curve.calculateCurvature(t) * centripetalCorrectionCoefficient)
                .scaledBy(velocity * velocity);
    }
}
