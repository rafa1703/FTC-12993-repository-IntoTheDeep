package org.firstinspires.ftc.teamcode.gvf.odo;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.gvf.utils.Encoder;
import org.firstinspires.ftc.teamcode.system.accessory.imu.ImuThread;
import org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware;

import java.util.Arrays;
import java.util.List;

public class TwoTrackingWheelLocalizer extends com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer
{
    private ImuThread imuThread;
    public static double TICKS_PER_REV = 2000;
    public static double WHEEL_RADIUS = 0.629921; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = -6.25; // X is the up and down direction
    public static double PARALLEL_Y =  4.35; // Y is the strafe direction

    public static double PERPENDICULAR_X = -6;
    public static double PERPENDICULAR_Y = -3.35;

    public static double X_MULTIPLIER = 1.0006645935143; // Multiplier in the X direction
    //0.9976947852, 0.9986984448, 1.005600550543

    public static double Y_MULTIPLIER = 1.000700204928; // Multiplier in the Y direction
    //1.0002644668732, 1.0002524590385, 1.001583688872
    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private Encoder parallelEncoder, perpendicularEncoder;


    public TwoTrackingWheelLocalizer(HardwareMap hardwareMap, ImuThread imuThread, LinearOpMode opMode) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        this.imuThread = imuThread;
        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "BR"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "Intake"));

        // reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        //perpendicularEncoder.setDirection(Encoder.Direction.REVERSE);
        parallelEncoder.setDirection(Encoder.Direction.REVERSE);
        imuThread.initImuThread();
        imuThread.startThread(opMode);
    }
    public TwoTrackingWheelLocalizer(HardwareMap hardwareMap, ImuThread imuThread) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        this.imuThread = imuThread;
        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "BR"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "Intake"));

        // reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        //perpendicularEncoder.setDirection(Encoder.Direction.REVERSE);
        parallelEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public TwoTrackingWheelLocalizer(GeneralHardware hardware) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        //this.imuThread = hardware.imu;
        parallelEncoder = hardware.perpendicularOdo;
        perpendicularEncoder = hardware.parallelOdo;

        // reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return imuThread.getImuAngle();
    }

    @Override
    public Double getHeadingVelocity() {
        return imuThread.getImuExternalVel();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        //  If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getRawVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getRawVelocity()) * Y_MULTIPLIER
        );
    }

}
