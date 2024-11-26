package org.firstinspires.ftc.teamcode.opmode.auto;

import org.firstinspires.ftc.teamcode.gvf.trajectories.BezierCurveTrajectorySegment;
import org.firstinspires.ftc.teamcode.gvf.trajectories.Trajectory;
import org.firstinspires.ftc.teamcode.gvf.trajectories.TrajectoryBuilder;
import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.opencv.core.Point;

public class PathsFarExtra
{

    public Trajectory
            preloadTrajectory, // preload deposit
            submersibleToFirstEjection,
            firstSamplePickup, secondEjection,
            secondSamplePickup, thirdEjection,
            thirdSamplePickup, forthEjection,
            firstIntake,firstDeposit, // cycle 1
            secondIntake, secondDeposit, // cycle 2
            thirdIntake, thirdDeposit, // cycle 3
            forthIntake, forthDeposit, // cycle 4
            fifthIntake, fifthDeposit, // cycle 5
            parkTrajectory;
    public Pose farStartPose = new Pose(7.2, -62.5, Math.toRadians(90));;
    public PathsFarExtra()
    {
        preloadTrajectory = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(7.2, -62.5),
                        new Point(7.2, -25)
                )
                .addFinalPose(7.2, -25, Math.toRadians(90))
                .build();

        submersibleToFirstEjection = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(7.2, -30),
                        new Point(7.2, -48),
                        new Point(26, -51)

                )
                .addFinalPose(18, -39, Math.toRadians(-20))
                .build();

        firstSamplePickup = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(26, -51),
                        new Point(30, -41)
                )
                .addFinalPose(18, -39, Math.toRadians(30))
                .build();

        secondEjection = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(30, -41),
                        new Point(30.1, -41.1)
                )
                .addFinalPose(30.1, -41.1, Math.toRadians(-30))
                .build();

        secondSamplePickup = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(30.1, -41.1),
                        new Point(35, -36)
                )
                .addFinalPose(35, -36, Math.toRadians(20))
                .build();

        thirdEjection = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(35, -36),
                        new Point(35.1, -36.1)
                )
                .addFinalPose(35.1, -36.1, Math.toRadians(-35))
                .build();

        thirdSamplePickup = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(35.1, -36.1),
                        new Point(35.2, -36.2)
                )
                .addFinalPose(35.2, -36.2, Math.toRadians(20))
                .build();

        forthEjection = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(35.2, -36.2),
                        new Point(36, -36.3)
                )
                .addFinalPose(36, -36.3, Math.toRadians(-35))
                .build();

        firstIntake = new TrajectoryBuilder() // Spline heading
                .addBezierSegment(
                        new Point(35.3, -36.3),
                        new Point(36, -55)
                )
                .addFinalPose(36, -55, Math.toRadians(90))
                .build();
        firstDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(36, -55),
                        new Point(4, -54),
                        new Point(4.5, -30)

                )
                .addFinalPose(4.5, -30, Math.toRadians(90))
                .build();

        secondIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(4.5, -30),
                        new Point(4, -54),
                        new Point(34, -37),
                        new Point(36, -55)
                )
                .addFinalPose(36, -55, Math.toRadians(90))
                .build();

        secondDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(36, -55),
                        new Point(2, -54),
                        new Point(2.5, -30)

                )
                .addFinalPose(2.5, -30, Math.toRadians(90))
                .build();

        thirdIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(2.5, -30),
                        new Point(2, -54),
                        new Point(34, -37),
                        new Point(36, -55)
                )
                .addFinalPose(36, -55, Math.toRadians(90))
                .build();

        thirdDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(36, -55),
                        new Point(0, -54),
                        new Point(0.5, -30)

                )
                .addFinalPose(0.5, -30, Math.toRadians(90))
                .build();

        forthIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(0.5, -30),
                        new Point(0, -54),
                        new Point(34, -37),
                        new Point(36, -55)
                )
                .addFinalPose(36, -55, Math.toRadians(90))
                .build();

        forthDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(36, -55),
                        new Point(-2, -54),
                        new Point(-2.5, -30)

                )
                .addFinalPose(-2.5, -30, Math.toRadians(90))
                .build();

        fifthIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(-2.5, -30),
                        new Point(-2, -54),
                        new Point(34, -37),
                        new Point(36, -55)
                )
                .addFinalPose(36, -55, Math.toRadians(90))
                .build();

        fifthDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(36, -55),
                        new Point(-4, -54),
                        new Point(-4.5, -30)

                )
                .addFinalPose(-4.5, -30, Math.toRadians(90))
                .build();

        parkTrajectory = new TrajectoryBuilder() // tangent reversed should be faster or we can spline heading (so we are looking at other alliance samples)
                .addBezierSegment(
                        new Point(0.5, -25),
                        new Point(46, -60)

                )
                .addFinalPose(46, -60, Math.toRadians(90))
                .build();
    }
}
