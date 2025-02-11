package org.firstinspires.ftc.teamcode.opmode.auto.paths;

import org.firstinspires.ftc.teamcode.gvf.trajectories.Trajectory;
import org.firstinspires.ftc.teamcode.gvf.trajectories.TrajectoryBuilder;
import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.opencv.core.Point;

public class SpecAutoPath
{
    public Trajectory
            preloadTrajectory, // preload deposit
            subToHpAndIntake, hpToSubIntake,
            subToSamples, thirdSampleToIntake,
            firstDepositWhileTurning, firstIntake, // cycle 1
            secondDeposit, secondIntake, // cycle 2
            thirdDeposit, thirdIntake, // cycle 3
            forthDeposit, forthIntake, // cycle 4
            fifthDeposit, // cycle 5
            parkTrajectory;

    public Pose farStartPose = new Pose(7.2, -62.5, Math.toRadians(90));
    public SpecAutoPath()
    {
        preloadTrajectory = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(7.2, -62.5),
                        new Point(0, -28.8)
                )
                .addFinalPose(0, -28.8, Math.toRadians(90))
                //.addTrajectoryTimeOut(300)
                .build();

        subToHpAndIntake = new TrajectoryBuilder() // spline heading
            .addBezierSegment(
                    new Point(0, -28.8),
                    //new Point(0, -26.8),
                    new Point(32, -59)
            )
            .addFinalPose(32, -59, Math.toRadians(135))
            .build();

        hpToSubIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(32, -59),
                        new Point(0, -28.8)
                        //new Point(0, -26.8),
                )
                .addFinalPose(32, -59, Math.toRadians(90))
                .build();

        subToSamples = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(0, -28.8),
                        new Point(0, -48),
                        new Point(35, -48)
                        //new Point(0, -26.8),
                )
                .addBezierSegment(0.15,
                        new Point(35, -48),
                        new Point(62, -48)
                )
                .addFinalPose(62, -48, Math.toRadians(90))
                .build();

        thirdSampleToIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(62, -48),
                        new Point(62, -61)
                )
                .addFinalPose(62, -61, Math.toRadians(90))
                .build();

        firstDepositWhileTurning = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(62, -61),
                        new Point(9, -61),
                        new Point(9, -38)
                )
                .addFinalPose(9, -38, Math.toRadians(-35))
                .build();

        firstIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(9, -38),
                        new Point(24, -48)
                )
                .addFinalPose(24, -48, Math.toRadians(-35))
                .build();

        secondDeposit = new TrajectoryBuilder() // spline heading
            .addBezierSegment(
                    new Point(24, -48),
                    //new Point(16, -48),
                    new Point(9, -38)

            )
            .addFinalPose(9, -38, Math.toRadians(-35)) // this might need to be at a lower angle
            .build();

        secondIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(9, -38),
                        new Point(24, -48)
                )
                .addFinalPose(24, -48, Math.toRadians(-35))
                .build();

        thirdDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(24, -48),
                        //new Point(16, -48),
                        new Point(9, -38)

                )
                .addFinalPose(9, -38, Math.toRadians(-35)) // this might need to be at a lower angle
                .build();

        thirdIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(9, -38),
                        new Point(24, -48)
                )
                .addFinalPose(24, -48, Math.toRadians(-35))
                .build();

        forthDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(24, -48),
                        //new Point(16, -48),
                        new Point(9, -38)

                )
                .addFinalPose(9, -38, Math.toRadians(-35)) // this might need to be at a lower angle
                .build();

        forthIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(9, -38),
                        new Point(24, -48)
                )
                .addFinalPose(24, -48, Math.toRadians(-35))
                .build();

        fifthDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(24, -48),
                        //new Point(16, -48),
                        new Point(9, -38)

                )
                .addFinalPose(9, -38, Math.toRadians(-35)) // this might need to be at a lower angle
                .build();

        parkTrajectory = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(9, -38),
                        new Point(46, -60)

                )
                .addFinalPose(46, -60, Math.toRadians(0))
                .build();
    }
}
