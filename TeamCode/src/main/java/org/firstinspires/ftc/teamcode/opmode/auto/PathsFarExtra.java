package org.firstinspires.ftc.teamcode.opmode.auto;

import org.firstinspires.ftc.teamcode.gvf.trajectories.BezierCurveTrajectorySegment;
import org.firstinspires.ftc.teamcode.gvf.trajectories.Trajectory;
import org.firstinspires.ftc.teamcode.gvf.trajectories.TrajectoryBuilder;
import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.opencv.core.Point;

public class PathsFarExtra
{

    public Trajectory firstOldPlusDragDLine;
    public Trajectory firstDLine;
    public Trajectory firstDLineInterpolation;
    public Trajectory subToFirstE, firstSP, secondE, secondSP, thirdE, thirdSP, forthE, SPFromSub;

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

    public Trajectory firstIntakePart1, firstIntakePart2; // this might not be needed
    public Pose farStartPose = new Pose(7.2, -62.5, Math.toRadians(90));
    public PathsFarExtra()
    {
        preloadTrajectory = new TrajectoryBuilder() // spline heading, this was tuned in field
                .addBezierSegment(
                        new Point(7.2, -62.5),
                        new Point(4, -30),
                        new Point(4, -26.8)
                )
                .addFinalPose(4, -26.8, Math.toRadians(90))
                .build();

        submersibleToFirstEjection = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(3, -30),
                        new Point(3, -48),
                        new Point(26, -51)

                )
                .addFinalPose(26, -51, Math.toRadians(-20))
                .build();
        subToFirstE = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(3, -30),
                        new Point(3, -42),
                        new Point(31, -42)

                )
                .addFinalPose(31, -42, Math.toRadians(-40))
                .build();

        firstSamplePickup = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(26, -51),
                        new Point(28, -38)
                )
                .addFinalPose(28, -38, Math.toRadians(32))
                .build();

        firstSP = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(31, -42),
                        new Point(31.1, -42.1)
                )
                .addFinalPose(31.1, -42.1, Math.toRadians(43))
                .build();
        SPFromSub = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(3, -30),
                        new Point(3, -48),
                        new Point(31.1, -42.1)
                )
                .addFinalPose(31.1, -42.1, Math.toRadians(43))
                .build();

        secondEjection = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(30, -41),
                        new Point(30.1, -44)
                )
                .addFinalPose(30.1, -44, Math.toRadians(-30))
                .build();

        secondE = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(31, -42),
                        new Point(31.2, -42.2)
                )
                .addFinalPose(31.2, -42.2, Math.toRadians(-40))
                .build();

        secondSamplePickup = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(30.1, -41.1),
                        new Point(36.5, -36)
                )
                .addFinalPose(36.5, -36, Math.toRadians(26.5))
                .build();

        secondSP = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(31.2, -42.2),
                        new Point(44, -35)
                )
                .addFinalPose(44, -35, Math.toRadians(26))
                .build();

        thirdEjection = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(35, -36),
                        new Point(35.1, -38.5)
                )
                .addFinalPose(35.1, -38.5, Math.toRadians(-45))
                .build();

        thirdE = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(44, -35),
                        new Point(44.1, -35.1)
                )
                .addFinalPose(44.1, -35.1, Math.toRadians(-50))
                .build();

        thirdSamplePickup = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(35.1, -36.1),
                        new Point(42, -36.2)
                )
                .addFinalPose(42, -36.2, Math.toRadians(23))
                .build();

        thirdSP = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(44.1, -35.1),
                        new Point(44.2, -35.2)
                )
                .addFinalPose(44.2, -35.2, Math.toRadians(23.5))
                .build();

        forthEjection = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(35.2, -36.2),
                        new Point(36, -39)
                )
                .addFinalPose(36, -39, Math.toRadians(-50))
                .build();

        forthE = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(44.2, -35.2),
                        new Point(44.3, -35.3)
                )
                .addFinalPose(44.3, -35.3, Math.toRadians(-47))
                .build();

        firstIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(36, -39),
                        new Point(37, -40.3),
                        new Point(37.8, -50)
                )
                .addBezierSegment(0.5, // keep in mind here that the x pos should be the same so it is a line
                        new Point(37.8, -50),
                        new Point(37.8, -57.1)
                )
                .addFinalPose(37.8, -57.1, Math.toRadians(90))
                .build();

        firstIntakePart1 =
                new TrajectoryBuilder() // spline heading
                        .addBezierSegment(
                                new Point(36, -39),
                                new Point(37, -40.3),
                                new Point(36.7, -50)
                        )
                        .addFinalPose(36.7, -50, Math.toRadians(90))
                        .build();
        firstIntakePart2 = new TrajectoryBuilder() // spline heading
                .addBezierSegment(0.9, // keep in mind here that the x pos should be the same so it is a line
                        new Point(37.3, -50),
                        new Point(36.7, -57.5)
                )
                .addFinalPose(36.7, -57.5, Math.toRadians(90))
                .build();

        firstDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(1,
                        new Point(36.7, -57.5),
                        new Point(2, -59),
                        new Point(2.5, -35)

                )
                .addBezierSegment( 0.85,
                        new Point(2.5, -35),
                        new Point(2.5, -27.1)

                )
                .addFinalPose(2.5, -27.1, Math.toRadians(90))
                .build();
        /*firstOldPlusDragDLine = new TrajectoryBuilder() // spline heading
                .addBezierSegment(1,
                        new Point(36.7, -57.5),
                        new Point(2, -59),
                        new Point(2.5, -35)

                )
                .addBezierSegment( 0.85,
                        new Point(2.5, -35),
                        new Point(2.5, -27.1)

                )
                .addBezierSegment( 0.85,
                        new Point(2.5, -27.1),
                        new Point(-4, -27.1)

                )
                .addFinalPose(-4, -27.1, Math.toRadians(90))
                .build();

        firstDLine = new TrajectoryBuilder() // spline heading, straight lines
                .addBezierSegment(
                        new Point(37.3, -57.5),
                        new Point(0, -37)
                )
                .addBezierSegment(
                        new Point(0, -37),
                        new Point(-1.5, -27.1)
                )
                .addFinalPose(-1.5, -28, Math.toRadians(90))
                .build();
        firstDLineInterpolation = new TrajectoryBuilder() // spline heading, straight lines
                .addBezierSegment(
                        new Point(37.3, -57.5),
                        new Point(8, -45),
                        new Point(2, -30)
                )
                .addBezierSegment(0.8,
                        new Point(2, -30),
                        new Point(-0.5, -27.1)
                )
                .addFinalPose(-0.5, -27.1, Math.toRadians(90))
                .build();*/

        secondIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        1,
                        new Point(0.5, -30),
                        new Point(0, -54),
                        new Point(39.9, -37),
                        new Point(39.9, -57.1)
                )
                .addFinalPose(39.9, -57.1, Math.toRadians(90))
                .build();

        secondDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(1,
                        new Point(36.7, -57.5),
                        new Point(-0, -59),
                        new Point(-0.5, -35)

                )
                .addBezierSegment( 0.85,
                        new Point(-0.5, -35),
                        new Point(-0.5, -27.1)

                )
                .addFinalPose(-0.5, -27.1, Math.toRadians(90))
                .build();

        thirdIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        1,
                        new Point(-2.5, -30),
                        new Point(-2, -54),
                        new Point(39.9, -37),
                        new Point(39.9, -57.1)
                )
                .addFinalPose(39.9, -57.1, Math.toRadians(90))
                .build();

        thirdDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(1,
                        new Point(36.7, -57.5),
                        new Point(-1, -59),
                        new Point(-1.5, -35)

                )
                .addBezierSegment( 0.85,
                        new Point(-1.5, -35),
                        new Point(-1.5, -27.1)

                )
                .addFinalPose(-1.5, -27.1, Math.toRadians(90))
                .build();

        forthIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        1,
                        new Point(-2.5, -30),
                        new Point(-2, -54),
                        new Point(39.9, -37),
                        new Point(39.9, -57.1)
                )
                .addFinalPose(39.9, -57.1, Math.toRadians(90))
                .build();

        forthDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(1,
                        new Point(36.7, -57.5),
                        new Point(-4, -59),
                        new Point(-4.5, -35)

                )
                .addBezierSegment( 0.85,
                        new Point(-4.5, -35),
                        new Point(-4.5, -27.1)

                )
                .addFinalPose(-4.5, -27.1, Math.toRadians(90))
                .build();

        fifthIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        1,
                        new Point(-2.5, -30),
                        new Point(-2, -54),
                        new Point(39.9, -37),
                        new Point(39.9, -57.1)
                )
                .addFinalPose(39.9, -57.1, Math.toRadians(90))
                .build();

        fifthDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(1,
                        new Point(36.7, -57.5),
                        new Point(-6, -59),
                        new Point(-6.5, -35)

                )
                .addBezierSegment( 0.85,
                        new Point(-6.5, -35),
                        new Point(-6.5, -27.1)

                )
                .addFinalPose(-6.5, -27.1, Math.toRadians(90))
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
