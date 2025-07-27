package org.firstinspires.ftc.teamcode.opmode.auto.paths;

import org.firstinspires.ftc.teamcode.gvf.trajectories.Trajectory;
import org.firstinspires.ftc.teamcode.gvf.trajectories.TrajectoryBuilder;
import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.opencv.core.Point;

public class SampleAutoPathNew
{
    public Trajectory
            preloadTrajectory, // preload deposit
            hpIntake, hpDeposit,
            firstIntake, firstDeposit,
            secondIntake, secondDeposit,
            thirdIntake, thirdDeposit,
            submersibleIntake, forthDeposit,
            submersibleIntakeSecond, fifthDeposit,
            submersibleIntakeThird, sixthDeposit,
            submersibleIntakeForth, seventhDeposit,
            submersibleIntakeFifth, eighthDeposit,
            parkTrajectory;
    public Pose closeStartPose = new Pose(-39, -62.5, Math.toRadians(90));
    public SampleAutoPathNew()
    {
        preloadTrajectory = new TrajectoryBuilder() // spline heading
                .addBezierSegment(0.7, // 0.4 works but there we wait until the trajectory ended
                        new Point(-39, -62.5),
                        new Point(-59.3, -55.2)
                )
                .addFinalPose(-59.3, -55.2, Math.toRadians(62))
                .build();
//                preloadTrajectory = new TrajectoryBuilder() // spline heading
//            .addBezierSegment(0.27,
//                    new Point(-39, -62.5),
//                    new Point(-50, -62.5)
//            )
//            .addFinalPose(-50, -62.5, Math.toRadians(0))
//            .build();

        hpIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(-57, -61.5),
                        new Point(12, -61.5)
                )
                .addFinalPose(12, -61.5, Math.toRadians(0))
                .build();

        hpDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(10, -62.5),
                        new Point(-57, -55)
                )
                .addFinalPose(-57, -55, Math.toRadians(75))
                .build();

        firstIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(0.8,
                        new Point(-56, -55),
                        new Point(-56.1, -53.8)
                )
                .addFinalPose(-56.1, -53.8, Math.toRadians(70))
                .build();

        firstDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(-56, -52.9),
                        new Point(-56, -55)
                )
                .addFinalPose(-56, -55, Math.toRadians(70))
                .build();

        secondIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(0.7,
                        new Point(-57, -55),
                        new Point(-57.5, -54.3)
                )
                .addFinalPose(-57.5, -54.3, Math.toRadians(90))
                .build();

        secondDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(0.8,
                        new Point(-57.1, -54),
                        new Point(-56, -55)
                )
                .addFinalPose(-56, -55, Math.toRadians(90))
                .build();

        thirdIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(0.7,
                        new Point(-57.1, -55),
                        new Point(-60, -52.3)
                )
                .addFinalPose(-60, -52.3, Math.toRadians(110))
                .build();

        thirdDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(-60, -52),
                        new Point(-56, -55)
                )
                .addFinalPose(-56, -55, Math.toRadians(65))
                .build();

        // idk if we will use this
        submersibleIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(-62, -64),
                        new Point(-37, -12),
                        new Point(-28, -12)
                )
                .addFinalPose(-28, -12, Math.toRadians(0))
                .build();


//                new TrajectoryBuilder() // spline heading
//                .addBezierSegment(
//                        new Point(-62, -64),
//                        new Point(-48, -9),
//                        new Point(-22, -11)
//                )
//                .addFinalPose(-22, -11, Math.toRadians(0))
//                .build();

        forthDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(0.7,
                        new Point(-22, -11),
                        new Point(-48, -9),
                        new Point(-57, -55)
                )
                .addFinalPose(-57, -55, Math.toRadians(90))
                .build();
        submersibleIntakeSecond = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(-62, -64),
//                        new Point(-48, -9),
                        new Point(-28, -12)
                )
                .addFinalPose(-28, -12, Math.toRadians(0))
                .build();
        fifthDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(0.7,
                        new Point(-22, -11),
                        new Point(-48, -9),
                        new Point(-57, -55)
                )
                .addFinalPose(-57, -55, Math.toRadians(90))
                .build();
        submersibleIntakeThird = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(-62, -64),
//                        new Point(-48, -9),
                        new Point(-28, -6)
                )
                .addFinalPose(-28, -12, Math.toRadians(0))
                .build();
        sixthDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(0.7,
                        new Point(-22, -11),
                        new Point(-48, -9),
                        new Point(-57, -55)
                )
                .addFinalPose(-57, -55, Math.toRadians(90))
                .build();
        submersibleIntakeForth = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(-62, -64),
//                        new Point(-48, -9),
                        new Point(-28, -4)
                )
                .addFinalPose(-28, -12, Math.toRadians(0))
                .build();
        seventhDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(0.7,
                        new Point(-22, -11),
                        new Point(-48, -9),
                        new Point(-57, -55)
                )
                .addFinalPose(-57, -55, Math.toRadians(90))
                .build();
        submersibleIntakeFifth = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(-62, -64),
//                        new Point(-48, -9),
                        new Point(-28, -2)
                )
                .addFinalPose(-28, -12, Math.toRadians(0))
                .build();
        // wtf is eighth
        eighthDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(0.7,
                        new Point(-22, -11),
                        new Point(-48, -9),
                        new Point(-57, -55)
                )
                .addFinalPose(-57, -55, Math.toRadians(90))
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
