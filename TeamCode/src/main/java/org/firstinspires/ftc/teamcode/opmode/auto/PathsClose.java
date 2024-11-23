package org.firstinspires.ftc.teamcode.opmode.auto;

import org.firstinspires.ftc.teamcode.gvf.trajectories.BezierCurveTrajectorySegment;
import org.firstinspires.ftc.teamcode.gvf.trajectories.Trajectory;
import org.firstinspires.ftc.teamcode.gvf.trajectories.TrajectoryBuilder;
import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.opencv.core.Point;

public class PathsClose
{
    public Trajectory
            preloadTrajectory, // preload deposit
            submersibleToSamplesTrajectory, firstSampleToHP,
            hpToSecondSample, secondSampleToHP,
            hpToThirdSample,
            thirdSampleToHPAndIntake, firstDeposit, // cycle 1
            firstIntake, secondDeposit, // cycle 2
            secondIntake, thirdDeposit, // cycle 3
            thirdIntake, forthDeposit, // cycle 4
            parkTrajectory;
    public Pose closeStartPose;
    public PathsClose()
    {
        preloadTrajectory = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(7.2, -62.5),
                        new Point(7.2, -25)
                )
                .addFinalPose(7.2, -25, Math.toRadians(90))
                .build();
        submersibleToSamplesTrajectory = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(7.2, -25),
                        new Point(14, -35),
                        new Point(19, -36)
                )
                .addBezierSegment(
                        new Point(19, -36),
                        new Point(43, -43),
                        new Point(35, -11),
                        new Point(48, -13)
                )
                .addFinalPose(48, -13, Math.toRadians(90))
                .build();

        firstSampleToHP = new TrajectoryBuilder() // tangent
                .addBezierSegment(
                        new Point(48, -13),
                        new Point(48, -57)
                )
                .addFinalPose(48, -57, Math.toRadians(90))
                .build();

        hpToSecondSample = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(48, -57),
                        new Point(48, -30)
                )
                .addBezierSegment(
                        new Point(48, -30),
                        new Point(48, -12),
                        new Point(58, -13)
                )
                .addFinalPose(58, -13, Math.toRadians(90))
                .build();

        secondSampleToHP = new TrajectoryBuilder() // splineHeading
                .addBezierSegment(
                        hpToSecondSample.getFinalPose().toPoint(),
                        new Point(58, -57)
                )
                .addFinalPose(58, -57, Math.toRadians(90))
                .build();

        hpToThirdSample = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(58, -57),
                        new Point(58, -30)
                )
                .addBezierSegment(
                        new Point(58, -30),
                        new Point(58, -12),
                        new Point(65, -13)
                )
                .addFinalPose(65, -13, Math.toRadians(90))
                .build();

        thirdSampleToHPAndIntake = new TrajectoryBuilder() // spline heading
                // might change this to a full speed line and just wait until we are stopped
                .addBezierSegment(
                        new Point(65, -13),
                        new Point(65, -57)
                )
                .addSegment(new BezierCurveTrajectorySegment(
                        0.35,
                        new Point(65, -57),
                        new Point(65, -61)
                ))
                .addFinalPose(65, -61, Math.toRadians(90))
                .build();
        firstDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(65, -61),
                        new Point(6, -54),
                        new Point(6.5, -25)
                )
                .addFinalPose(6.5, -25, Math.toRadians(90))
                .build();
        firstIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(6.5, -25),
                        new Point(6, -54),
                        new Point(34, -37),
                        new Point(36, -55)
                )
                .addSegment(
                        new BezierCurveTrajectorySegment(
                                0.35,
                                new Point(36, -55),
                                new Point(36, -61)
                        )
                )
                .addFinalPose(36, -61, Math.toRadians(90))
                .build();
        secondDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(36, -61),
                        new Point(4, -54),
                        new Point(4.5, -25)

                )
                .addFinalPose(4.5, -25, Math.toRadians(90))
                .build();
        secondIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(4.5, -25),
                        new Point(4, -54),
                        new Point(34, -37),
                        new Point(36, -55)
                )
                .addSegment(
                        new BezierCurveTrajectorySegment(
                                0.35,
                                new Point(36, -55),
                                new Point(36, -61)
                        )
                )
                .addFinalPose(36, -61, Math.toRadians(90))
                .build();
        thirdDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(36, -61),
                        new Point(2, -54),
                        new Point(2.5, -25)

                )
                .addFinalPose(2.5, -25, Math.toRadians(90))
                .build();
        thirdIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(2.5, -25),
                        new Point(2, -54),
                        new Point(34, -37),
                        new Point(36, -55)
                )
                .addSegment(
                        new BezierCurveTrajectorySegment(
                                0.35,
                                new Point(36, -55),
                                new Point(36, -61)
                        )
                )
                .addFinalPose(36, -61, Math.toRadians(90))
                .build();
        forthDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(36, -61),
                        new Point(0, -54),
                        new Point(0.5, -25)

                )
                .addFinalPose(0.5, -25, Math.toRadians(90))
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
