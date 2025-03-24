package org.firstinspires.ftc.teamcode.system.vision;

import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.firstinspires.ftc.teamcode.gvf.utils.Vector;

@SuppressWarnings("FieldCanBeLocal")
public class InverseKinematics
{
    // this is assumed with the camera being perpendicular and scanning the sub
    private final double h1 = 9.84; // height of the camera mount
    private final double h2 = 1.5; // height of the sample in the floor
    private final double a1 = -7; // camera mounting angle
    /** Takes ty from the limelight as a parameter, and returns a magnitude **/
    public double distanceToSample(double ty)
    {
        double Ty = Math.toRadians(ty);
        return (h2 - h1) / Math.tan(Math.toRadians(-15) + Ty);
    }
    public double horizontalDistanceToSample(double ty, double tx)
    {
        double y = distanceToSample(ty);
        double c = y / Math.cos(Math.toRadians(tx));
        return Math.sin(Math.toRadians(tx)) * c;
    }
    public Vector distanceToSample(double ty, double tx)
    {
        double Ty = Math.toRadians(ty);
        double y = (h2 - h1) / Math.tan(Math.toRadians(a1) + Ty);
        double Tx = Math.toRadians(tx);
        double c = y / Math.cos(Tx);
        double x = Math.sin(Tx) * c;
        return new Vector(x, y);
    }
    public Pose samplePos(double ty, double tx, Pose robotPose)
    {


        //  |----|
        //   / |
        //c /  | y
        // /---|
        //   x
        double y = distanceToSample(ty);
        double c = y / Math.cos(Math.toRadians(tx));
        double x = Math.sin(Math.toRadians(tx)) * c;
        return  robotPose.plus(new Pose(x, y ));
    }
    public Pose targetPoseAlignedWithSample(Pose currentPos, double distanceToSample, double tx)
    {
        tx = Math.toRadians(tx);
        Pose poseToSample = new Pose(Math.cos(tx) * distanceToSample, Math.sin(tx) * distanceToSample);
        return currentPos.plus(poseToSample);
    }
    public Pose targetPoseAimingToSample(Pose currentPose, double tx)
    {
        tx = Math.toRadians(tx);
        return currentPose.plus(new Pose(0, 0, tx));
    }
    public Pose targetPoseAlignOrNot(Pose currentPose, double tx, double ty)
    {
        if (Math.abs(tx) > 15) return targetPoseAlignedWithSample(currentPose, distanceToSample(ty), tx); // if the sample is too far we drive to it
        else return targetPoseAimingToSample(currentPose, tx); // else we turn to aim at it
    }

}
