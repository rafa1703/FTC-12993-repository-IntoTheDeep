package org.firstinspires.ftc.teamcode.opmode.auto.paths;

import org.firstinspires.ftc.teamcode.gvf.trajectories.Trajectory;
import org.firstinspires.ftc.teamcode.gvf.trajectories.TrajectoryBuilder;
import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.opencv.core.Point;

public class SpecAutoPath
{
    public Trajectory
            preloadTrajectory, // preload deposit
            subToHpAndIntake,
            subToHp, hpIntake,  hpToSubIntake,
            subToHpExtendo,
            subToSamples, subDrop,
            firstSample, firstSampleDrop,
            secondSample, secondSampleDrop,
            thirdSample, thirdSampleDrop,
            thirdSampleToIntake,
            firstDepositWhileTurning, firstIntake, // cycle 1
            secondDeposit, secondIntake, // cycle 2
            thirdDeposit, thirdIntake, // cycle 3
            forthDeposit, forthIntake, // cycle 4
            fifthDeposit, fifthIntake, // cycle 5
            parkTrajectory;

    public Pose farStartPose = new Pose(7.2, -62.5, Math.toRadians(90));
    public SpecAutoPath()
    {
        preloadTrajectory = new TrajectoryBuilder() // spline heading
                .addBezierSegment(0.5,
                        new Point(7.2, -62.5),
                        new Point(0, -30)
                )
                .addFinalSpeed(0.5)
                .addFinalPose(0, -30, Math.toRadians(90))
                //.addTrajectoryTimeOut(300)
                .build();

        subToHp = new TrajectoryBuilder() // spline heading
            .addBezierSegment(0.8,
                    new Point(0, -28.8),
                    new Point(36, -60)
            )
//            .addTrajectoryTimeOut(850)
            .addFinalPose(36, -60, Math.toRadians(90))
            .build();

        hpIntake = new TrajectoryBuilder() // spline heading
            .addBezierSegment(0.7,
                    new Point(36, -60),
                    new Point(36, -62.5)
            )
            .addFinalPose(new Pose(36, -62.5, Math.toRadians(90)))
            .build();
        subToHpAndIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(0.8,
                        new Point(0, -28),
                        new Point(0, -54),
                        new Point(36, -37),
                        new Point(36, -63)
                )
//                .addFinalSpeed(0.3)
                .addFinalPose(36, -63, Math.toRadians(90))
                .build();

        hpToSubIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(32, -59),
                        new Point(0, -28.5)
                        //new Point(0, -26.8),
                )
                .addFinalSpeed(0.5)
                .addFinalPose(0, -28.5, Math.toRadians(90))
                .build();

        subToHpExtendo = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(32, -59),
                        new Point(0, -28.8)
                        //new Point(0, -26.8),
                )
                .addFinalSpeed(0.5)
                .addFinalPose(0, -28.8, Math.toRadians(90))
                .build();

        subToSamples = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(0, -28.8),
                        new Point(0, -51),
                        new Point(35, -52)
                        //new Point(0, -26.8),
                )
                .addBezierSegment(0.2,
                        new Point(35, -52),
                        new Point(62, -52)
                )
                .addFinalPose(62, -52, Math.toRadians(90))
                .build();

//        subDropAndFirstSample = new TrajectoryBuilder() // spline heading
//                .addBezierSegment(
//                        new Point(0, -28.8),
//                        new Point(0, -51),
//                        new Point(48.5, -52)
//                )
//                .addFinalPose(48.5, -52, Math.toRadians(90))
//                .build();
        subDrop = new TrajectoryBuilder() // spline heading
                .addBezierSegment(0.8,
                        new Point(0, -28.8),
                        new Point(0, -52),
                        new Point(43, -52)
                        //new Point(0, -26.8),
                )
                .addFinalSpeed(0.5)
                .addFinalPose(43, -52, Math.toRadians(90))
                .build();
        firstSample = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(43, -52),
                        new Point(45.5, -52.1)
                )
                .addFinalPose(45.5, -52.1, Math.toRadians(90))
                .build();
        firstSampleDrop = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(45.5, -52),
                        new Point(50, -52)

                )
                .addFinalPose(50, -52, Math.toRadians(90))
                .build();
        secondSample = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(50, -52),
                        new Point(54, -52)
                )
                .addFinalPose(54, -52.1, Math.toRadians(90))
                .build();
        secondSampleDrop = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(54, -52),
                        new Point(58, -52)
                )
                .addFinalPose(58, -52, Math.toRadians(90))
                .build();
        thirdSample = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(58, -52),
                        new Point(60.5, -52)
                )
                .addFinalPose(60.5, -52, Math.toRadians(80))
                .build();
        thirdSampleToIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(60.5, -52.1),
                        new Point(60.5, -63.3)
                )
                .addFinalPose(60.5, -63.3, Math.toRadians(90))
                .build();

        firstDepositWhileTurning = new TrajectoryBuilder() // spline heading
                .addBezierSegment(0.6,
                        new Point(62, -63.3),
                        new Point(39, -46),
                        new Point(9, -63.3),
                        new Point(9, -35.7)
                )
                .addFinalPose(9, -35.7, Math.toRadians(-35))
                .build();

        firstIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(0.5,
                        new Point(9, -38),
                        new Point(24, -44)
                )
                .addFinalPose(24, -44, Math.toRadians(-35))
                .build();

        secondDeposit = new TrajectoryBuilder() // spline heading
            .addBezierSegment(
                    new Point(24, -48),
                    //new Point(16, -48),
                    new Point(9, -35.7)

            )
            .addFinalPose(9, -35.7, Math.toRadians(-35)) // this might need to be at a lower angle
            .build();

        secondIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(0.5,
                        new Point(9, -38),
                        new Point(24, -44)
                )
                .addFinalPose(24, -44, Math.toRadians(-35))
                .build();

        thirdDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(24, -48),
                        //new Point(16, -48),
                        new Point(9, -35.7)

                )
                .addFinalPose(9, -35.7, Math.toRadians(-35)) // this might need to be at a lower angle
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
                        new Point(9, -35.7)

                )
                .addFinalPose(9, -35.7, Math.toRadians(-35)) // this might need to be at a lower angle
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
                        new Point(9, -35.7)

                )
                .addFinalPose(9, -35.7, Math.toRadians(-35)) // this might need to be at a lower angle
                .build();
        fifthIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(9, -38),
                        new Point(24, -48)
                )
                .addFinalPose(24, -48, Math.toRadians(-35))
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
