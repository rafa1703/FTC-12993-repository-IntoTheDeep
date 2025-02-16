package org.firstinspires.ftc.teamcode.opmode.auto.paths;

import org.firstinspires.ftc.teamcode.gvf.trajectories.Trajectory;
import org.firstinspires.ftc.teamcode.gvf.trajectories.TrajectoryBuilder;
import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.opencv.core.Point;

public class SampleAutoPath
{
    public Trajectory
            preloadTrajectory, // preload deposit
            hpIntake, hpDeposit,
            firstIntake, firstDeposit,
            secondIntake, secondDeposit,
            thirdIntake, thirdDeposit,
            submersibleIntake, forthDeposit,
            submersibleIntakeSecond, fifthDeposit,
            parkTrajectory;
    public Pose closeStartPose = new Pose(-34, -62.5, Math.toRadians(90));
    public SampleAutoPath()
    {
        preloadTrajectory = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(-34, -62.5),
                        new Point(-57, -62.5)
                )
                .addFinalPose(-57, -62.5, Math.toRadians(0))
                .build();

        hpIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(-57, -62.5),
                        new Point(8.5, -62.5)
                )
                .addFinalPose(8.5, -62.5, Math.toRadians(0))
                .build();

        hpDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(8.5, -62.5),
                        new Point(-59, -54)
                )
                .addFinalPose(-59, -54, Math.toRadians(75))
                .build();

        firstIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(-59, -54),
                        new Point(-57, -53)
                )
                .addFinalPose(-57, -53, Math.toRadians(75))
                .build();

        firstDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(-57, -53),
                        new Point(-59, -54)
                )
                .addFinalPose(-59, -54, Math.toRadians(83))
                .build();

        secondIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(-59, -54),
                        new Point(-61, -52)

                )
                .addFinalPose(-61, -52, Math.toRadians(90))
                .build();

        secondDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(-62, -50),
                        new Point(-62, -54)
                )
                .addFinalPose(-62, -54, Math.toRadians(90))
                .build();

        thirdIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(-62, -54),
                        new Point(-60, -50)
                )
                .addFinalPose(-60, -50, Math.toRadians(100))
                .build();

        thirdDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(-60, -50),
                        new Point(-62, -54)
                )
                .addFinalPose(-62, -54, Math.toRadians(90))
                .build();

        // idk if we will use this
        submersibleIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(-62, -64),
                        new Point(-48, -9),
                        new Point(-22, -11)
                )
                .addFinalPose(-22, -11, Math.toRadians(0))
                .build();

        forthDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(-22, -11),
                        new Point(-48, -9),
                        new Point(-62.5, -64)
                )
                .addFinalPose(-62.5, -64, Math.toRadians(90))
                .build();
        submersibleIntakeSecond = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(-62, -64),
                        new Point(-48, -9),
                        new Point(-22, -11)
                )
                .addFinalPose(-22, -11, Math.toRadians(0))
                .build();
        fifthDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(-22, -11),
                        new Point(-48, -9),
                        new Point(-62.5, -64)
                )
                .addFinalPose(-62.5, -64, Math.toRadians(90))
                .build();

        parkTrajectory = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(-62, -64),
                        new Point(-48, -9),
                        new Point(-22, -11)
                )
                .addFinalPose(-22, -11, Math.toRadians(0))
                .build();
    }
}
