package org.firstinspires.ftc.teamcode.gvf;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
    public double xOffset, yOffset, headingOffset;

    public static double filterParameter = 0.8;
    public LocalizerPinpoint(GeneralHardware hardware, Pose startPose)
    {
        this.pose = startPose;
        this.localizer = hardware.pinpoint;
        localizer.setPosition(startPose.toPose2D());
    }
    public LocalizerPinpoint(GeneralHardware hardware)
    {
        this(hardware, new Pose());
    }
    public LocalizerPinpoint(GoBildaPinpointDriver driver)
    {
        this.pose = new Pose();
        this.localizer = driver;
        localizer.setPosition(pose.toPose2D());
    }

    @Deprecated
    public void setPose(Pose pose) {
        Pose2D pose2D = pose.toPose2D();
        localizer.setPosition(pose2D);
        this.pose = pose;
    }

    public void setOffSet(double xOffset, double yOffset, double headingOffset)
    {
        this.xOffset = xOffset;
        this.yOffset = yOffset;
        this.headingOffset = headingOffset;
    }
    public void setOffSet(Pose pose)
    {
        this.setOffSet(pose.getX(), pose.getY(), pose.getHeading());
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

    public static double xDeceleration = 30.88, yDeceleration = 57.17;
    // x 30.88
    // y 57.17


    public Vector getVelocity(){
        return velocity;
    }
    public Vector getGlideDelta() {return driveBaseSVector;}
    // Cyliis cooked hard on this so...
    public void update() {
        if(!ENABLED) return;
        localizer.update();

        // This fixes the pose
        Pose2D pose2d = localizer.getPosition();
        pose = new Pose(pose2d.getX(DistanceUnit.INCH), pose2d.getY(DistanceUnit.INCH), pose2d.getHeading(AngleUnit.RADIANS));
        Vector vector = new Vector(pose.getX() , pose.getY(), pose.getHeading());
        double h = vector.getZ();
        vector = Vector.rotateBy(vector, -headingOffset); // zed dissapears here i pretty sure
        vector = new Vector(vector.getX(), vector.getY(), h + headingOffset);
        pose = new Pose(vector.getX() + xOffset, vector.getY()  + yOffset, vector.getZ());

        velocity = new Vector(
                xVelFilter.getValue(mmPerSecondToInPerSecond(localizer.getVelX())),
                yVelFilter.getValue(mmPerSecondToInPerSecond(localizer.getVelY())));
        driveBaseVelocity = Vector.rotateBy(velocity, -headingOffset);
        Vector sVector = new Vector(
                Math.signum(driveBaseVelocity.getX()) * Math.pow(driveBaseVelocity.getX(), 2) / (2.0 * xDeceleration),
                Math.signum(driveBaseVelocity.getY()) * Math.pow(driveBaseVelocity.getY(), 2) / (2.0 * yDeceleration));
        driveBaseSVector = sVector.rotated(0);
    }

    // sign of u * uˆ2 / 2a

    public double mmPerSecondToInPerSecond(double mm)
    {
        return mm / 25.4;
    }
    private double mmToIn(double mm)
    {
        return mm / 25.4;
    }
    private double inTomm(double in)
    {
        return in * 25.4;
    }
}
