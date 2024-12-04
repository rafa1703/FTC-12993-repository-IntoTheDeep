package org.firstinspires.ftc.teamcode.opmode.auto;

import org.firstinspires.ftc.teamcode.gvf.trajectories.Trajectory;
import org.firstinspires.ftc.teamcode.gvf.trajectories.TrajectoryBuilder;
import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.opencv.core.Point;

public class PathsClose
{
    public Trajectory
            preloadTrajectory, // preload deposit
            firstIntake, firstDeposit,
            secondIntake, secondDeposit,
            thirdIntake, thirdDeposit,
            submersibleIntake, forthDeposit,
            parkTrajectory;
    public Pose closeStartPose = new Pose(-34, -62.5, Math.toRadians(90));
    public PathsClose()
    {
        preloadTrajectory = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(-34, -62.5),
                        new Point(-62, -58.5)
                )
                .addFinalPose(-62, -58.5, Math.toRadians(45))
                .build();
        firstIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(-62, -57),
                        new Point(-62.3, -51)
                )
                .addFinalPose(-62.3, -51, Math.toRadians(70))
                .build();

        firstDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(-62.3, -53),
                        new Point(-62.5, -59)
                )
                .addFinalPose(-62.5, -59, Math.toRadians(45))
                .build();

        secondIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(-62.5, -59),
                        new Point(-62.5, -51),
                        new Point(-62.5, -48)
                )
                .addFinalPose(-62.5, -48, Math.toRadians(90))
                .build();

        secondDeposit = new TrajectoryBuilder() // spline heading
                // this might not work because of the heading
                .addBezierSegment(
                        new Point(-60, -46),
                        new Point(-60, -51),
                        new Point(-62.5, -59)
                )
                .addFinalPose(-62.5, -59, Math.toRadians(45))
                .build();

        thirdIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(-62.5, -59),
                        new Point(-60, -50)
                )
                .addFinalPose(-60, -50, Math.toRadians(115))
                .build();

        thirdDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(-60, -50),
                        new Point(-62.5, -58.5)
                )
                .addFinalPose(-62.5, -58.5, Math.toRadians(45))
                .build();
        // idk if we will use this
        submersibleIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(-62.5, -59),
                        new Point(-48, -7),
                        new Point(-22, -9)
                )
                .addFinalPose(-22, -9, Math.toRadians(90))
                .build();
        forthDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(-22, -9),
                        new Point(-48, -7),
                        new Point(-62.5, -59)
                )
                .addFinalPose(-62.5, -59, Math.toRadians(90))
                .build();
        parkTrajectory = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        0.9,
                        new Point(-62.5, -59),
                        new Point(-62.5, -10),
                        new Point(-24, -10)
                )
                .addFinalPose(-24, -10, Math.toRadians(0))
                .build();
    }
}
