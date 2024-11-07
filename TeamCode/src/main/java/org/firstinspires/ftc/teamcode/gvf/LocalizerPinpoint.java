package org.firstinspires.ftc.teamcode.gvf;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.gvf.odo.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.gvf.odo.TwoTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.gvf.utils.LowPassFilter;
import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.firstinspires.ftc.teamcode.gvf.utils.Vector;
import org.firstinspires.ftc.teamcode.system.accessory.imu.ImuThread;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

public class LocalizerPinpoint
{
    public static boolean ENABLED = true;
    private Pose pose;
    private GoBildaPinpointDriver localizer;
    private Vector velocity = new Vector();
    public Vector driveBaseVelocity = new Vector();
    public Vector driveBaseSVector = new Vector();

    public static double filterParameter = 0.8;
    public LocalizerPinpoint(GeneralHardware hardware, Pose startPose)
    {
        this.pose = startPose;
        this.localizer = hardware.pinpointLocalizer;
        localizer.setPosition(startPose.toPose2D());
    }
    public LocalizerPinpoint(GeneralHardware hardware)
    {
        this(hardware, new Pose());
    }
    @Deprecated
    public Pose getPredictedPose()
    {
        return new Pose();
    }

    public void setPose(Pose pose) {
        localizer.setPosition(pose.toPose2D());
        this.pose = pose;
    }

    public Pose getPoseEstimate() {
        return pose;
    }

    public Pose getPredictedPoseEstimate(){
        return new Pose(pose.getX() + driveBaseSVector.getX(), pose.getY() + driveBaseSVector.getY(), pose.getHeading());
    }

    public double getHeading(){
        return pose.getHeading();
    }


    private final LowPassFilter xVelFilter = new LowPassFilter(filterParameter, 0),
            yVelFilter = new LowPassFilter(filterParameter, 0);

    public static double xDeceleration = 100, yDeceleration = 150;

    public Vector getVelocity(){
        return velocity;
    }
    public Vector getGlideDelta() {return driveBaseSVector;}
    // Cyliis cooked hard on this so...
    public void update() {
        if(!ENABLED) return;
        localizer.update();
        Pose2D pose2d = localizer.getPosition();
        pose = new Pose(pose2d);
        velocity = new Vector(
                xVelFilter.getValue(localizer.getVelX()),
                yVelFilter.getValue(localizer.getVelY()));
        driveBaseVelocity = Vector.rotateBy(velocity, 0);
        Vector sVector = new Vector(
                Math.signum(driveBaseVelocity.getX()) * Math.pow(driveBaseVelocity.getX(), 2) / (2.0 * xDeceleration),
                Math.signum(driveBaseVelocity.getY()) * Math.pow(driveBaseVelocity.getY(), 2) / (2.0 * yDeceleration));
        driveBaseSVector = sVector.rotated(-pose.getHeading());
    }

    // sign of u * uˆ2 / 2a


}
