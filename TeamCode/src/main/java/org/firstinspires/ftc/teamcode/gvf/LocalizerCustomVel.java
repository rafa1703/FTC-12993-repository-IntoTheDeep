package org.firstinspires.ftc.teamcode.gvf;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.gvf.odo.TwoTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.gvf.utils.LowPassFilter;
import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.firstinspires.ftc.teamcode.gvf.utils.Vector;
import org.firstinspires.ftc.teamcode.system.accessory.imu.ImuThread;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

public class LocalizerCustomVel
{
    public static boolean ENABLED = true;
    private Pose pose;
    private com.acmerobotics.roadrunner.localization.Localizer localizer;
    private Vector velocity = new Vector();
    public Vector driveBaseVelocity = new Vector();
    public Vector driveBaseSVector = new Vector();
    private ElapsedTime timer = new ElapsedTime();
    private Pose lastPose;

    public static double filterParameter = 0.8;
    public LocalizerCustomVel(HardwareMap hardwareMap, Pose startingPose, LinearOpMode opMode)
    {
        this.pose = startingPose;
        this.localizer = new TwoTrackingWheelLocalizer(hardwareMap, new ImuThread(hardwareMap), opMode);
        localizer.setPoseEstimate(startingPose.toPose2d());

    }
    public LocalizerCustomVel(HardwareMap hardwareMap, Pose startingPose, ImuThread imuThread)
    {
        this.pose = startingPose;
        this.localizer = new TwoTrackingWheelLocalizer(hardwareMap, imuThread);
        localizer.setPoseEstimate(startingPose.toPose2d());

    }
    public LocalizerCustomVel(HardwareMap hardwareMap, Pose2d startingPose, LinearOpMode opMode)
    {
        this(hardwareMap, new Pose(startingPose.getX(), startingPose.getY(), startingPose.getHeading()), opMode);

    }

    public LocalizerCustomVel(HardwareMap hardwareMap, LinearOpMode opMode)
    {
        this(hardwareMap, new Pose(), opMode);
    }

    public LocalizerCustomVel(GeneralHardware hardware, Pose startPose)
    {
        this.pose = startPose;
        this.lastPose = startPose;
        this.localizer = new TwoTrackingWheelLocalizer(hardware);
        localizer.setPoseEstimate(pose.toPose2d());
        timer.reset();
    }
    public LocalizerCustomVel(GeneralHardware hardware)
    {
        this(hardware, new Pose());
    }
    @Deprecated
    public Pose getPredictedPose()
    {
        return new Pose();
    }

    public void setPose(Pose pose) {
        localizer.setPoseEstimate(new Pose2d(pose.getX(), pose.getY(), pose.getHeading()));
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
        Pose2d pose2d = localizer.getPoseEstimate();
        pose = new Pose(pose2d);
        velocity = new Vector(
                xVelFilter.getValue((pose.getX() - lastPose.getX()) / timer.seconds()),
                yVelFilter.getValue((pose.getY() - lastPose.getY()) / timer.seconds()));
        timer.reset();
        lastPose = pose;
        driveBaseVelocity = Vector.rotateBy(velocity, 0);
        Vector sVector = new Vector(
                Math.signum(driveBaseVelocity.getX()) * Math.pow(driveBaseVelocity.getX(), 2) / (2.0 * xDeceleration),
                Math.signum(driveBaseVelocity.getY()) * Math.pow(driveBaseVelocity.getY(), 2) / (2.0 * yDeceleration));
        driveBaseSVector = sVector.rotated(0);

    }

    // sign of u * uˆ2 / 2a


}
