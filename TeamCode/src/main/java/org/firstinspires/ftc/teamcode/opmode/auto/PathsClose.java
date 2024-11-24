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
            submersibleToFirstIntake, firstDeposit,
            secondIntake, secondDeposit,
            thirdIntake, thirdDeposit,
            submersibleIntake, forthDeposit,
            parkTrajectory;
    public Pose closeStartPose = new Pose(-10, -62.5);
    public PathsClose()
    {
        preloadTrajectory = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(-10, -62.5),
                        new Point(-10, -30)
                )
                .addFinalPose(-10, -30, Math.toRadians(180))
                .build();
        submersibleToFirstIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(-10, -30),
                        new Point(-16, -48),
                        new Point(-60, -53)
                )
                .addFinalPose(-60, -53, Math.toRadians(65))
                .build();

        firstDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(-60, -53),
                        new Point(-62.5, -59)
                )
                .addFinalPose(-62.5, -59, Math.toRadians(65))
                .build();

        secondIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(-62.5, -59),
                        new Point(-60, -51),
                        new Point(-60, -46)
                )
                .addFinalPose(-60, -46, Math.toRadians(90))
                .build();

        secondDeposit = new TrajectoryBuilder() // spline heading
                // this might not work because of the heading
                .addBezierSegment(
                        new Point(-60, -46),
                        new Point(-60, -51),
                        new Point(-62.5, -59)
                )
                .addFinalPose(-62.5, -59, Math.toRadians(90))
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
                        new Point(-62.5, -59)
                )
                .addFinalPose(-62.5, -59, Math.toRadians(90))
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
                        new Point(-62.5, -59),
                        new Point(-48, -9),
                        new Point(-22, -12)
                )
                .addFinalPose(-22, -12, Math.toRadians(0))
                .build();
    }
}
