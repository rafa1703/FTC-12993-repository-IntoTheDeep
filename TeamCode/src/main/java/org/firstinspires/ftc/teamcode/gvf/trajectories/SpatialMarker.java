package org.firstinspires.ftc.teamcode.gvf.trajectories;


import org.firstinspires.ftc.teamcode.gvf.utils.Pose;

public class SpatialMarker
{
    Callback callback;
    Pose spatialPoint;
    double t, u;
    public SpatialMarker(Pose pose, Callback callback)
    {
        this.callback = callback;
        spatialPoint = pose;
    }

}

