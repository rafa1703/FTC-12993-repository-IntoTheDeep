package org.firstinspires.ftc.teamcode.opmode.auto.paths;

import org.firstinspires.ftc.teamcode.gvf.trajectories.Trajectory;
import org.firstinspires.ftc.teamcode.gvf.trajectories.TrajectoryBuilder;
import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.firstinspires.ftc.teamcode.gvf.utils.Vector;
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
            sixthDeposit,
            parkTrajectory;

    public Pose farStartPose = new Pose(7.2, -62.5, Math.toRadians(90));
    public SpecAutoPath()
    {
        preloadTrajectory = new TrajectoryBuilder() // spline heading
                .addBezierSegment(1,
                        new Point(7.2, -62.5),
                        new Point(5, -31)
                )
                .addFinalSpeed(0.7)
                .addFinalPose(5, -31, Math.toRadians(90))
                //.addTrajectoryTimeOut(300)
                .build();

        subToHp = new TrajectoryBuilder() // spline heading
            .addBezierSegment(0.8,
                    new Point(5, -28.8),
                    new Point(36, -60)
            )
//            .addTrajectoryTimeOut(850)
            .addFinalPose(36, -60, Math.toRadians(90))
            .build();

        hpIntake = new TrajectoryBuilder() // spline heading
            .addBezierSegment(0.7,
                    new Point(36, -60),
                    new Point(36, -61.5)
            )
            .addFinalPose(new Pose(36, -61.5, Math.toRadians(90)))
            .build();
        subToHpAndIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(0.8,
                        new Point(0, -28),
                        new Point(0, -54),
                        new Point(36, -37),
                        new Point(36, -62.5)
                )
//                .addFinalSpeed(0.3)
                .addFinalPose(36, -62.5, Math.toRadians(90))
                .build();

        hpToSubIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(32, -59),
                        new Point(7, -43),
                        new Point(7, -31)
                        //new Point(0, -26.8),
                )
                .addFinalSpeed(0.7)
                .addFinalPose(7, -31, Math.toRadians(90))
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
                .addBezierSegment(0.9,
                        new Point(0, -28.8),
                        new Point(0, -52),
                        new Point(41, -52)
                        //new Point(0, -26.8),
                )
                .addFinalSpeed(0.5)
                .addFinalPose(41, -52, Math.toRadians(90))
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
                        new Point(45.5, -54),
                        new Point(51, -54)

                )
                .addFinalPose(51, -54, Math.toRadians(90))
                .build();
        secondSample = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(50, -52),
                        new Point(55.5, -52)
                )
                .addFinalPose(55.5, -52, Math.toRadians(90))
                .build();
        secondSampleDrop = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(55.5, -54),
                        new Point(58, -54)
                )
                .addFinalPose(58, -54, Math.toRadians(90))
                .build();
        thirdSample = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(58, -52),
                        new Point(60.5, -52)
                )
                .addFinalPose(60.5, -52, Math.toRadians(78))
                .build();
        thirdSampleDrop = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(60.5, -52),
                        new Point(55, -54)
                )
                .addFinalPose(55, -54, Math.toRadians(90))
                .build();
        thirdSampleToIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(60.5, -50),
                        new Point(36, -50),
                        new Point(36, -62.5)
                )
                .addFinalPose(36, -62.5, Math.toRadians(90))
                .build();
        Pose depositEndPose = new Pose(10.5, -31.8, Math.toRadians(90));
        firstDepositWhileTurning = new TrajectoryBuilder() // spline heading
                .addBezierSegment(1,
                        thirdSampleToIntake.getFinalPose().toPoint(),
//                        new Point(39, -46),
//                        new Point(9, -63.3),
                        depositEndPose.plus(new Pose(-4, 0, 0)).toPoint()
                )
                .addFinalPose(depositEndPose.plus(new Pose(-4, 0, 0)))
                .build();
        Pose intakeEndPose = new Pose(36, -62.5, Math.toRadians(90));
        firstIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(0.6,
                        new Point(0, -28),
                        new Point(0, -54),
                        new Point(36, -37),
                        new Point(36, -62)
                )
//                .addFinalSpeed(0.3)
                .addFinalPose(36, -62, Math.toRadians(90))
                .build();

        secondDeposit = new TrajectoryBuilder() // spline heading
            .addBezierSegment(1,
                    firstIntake.getFinalPose().toPoint(),
                    //new Point(16, -48),
                    depositEndPose.plus(new Pose(-9, 0, 0)).toPoint()
            )
            .addFinalPose(depositEndPose.plus(new Pose(-9, 0, 0))) // this might need to be at a lower angle
            .build();

        secondIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(0.6,
                        new Point(0, -28),
                        new Point(0, -54),
                        new Point(36, -37),
                        new Point(36, -62)
                )
//                .addFinalSpeed(0.3)
                .addFinalPose(36, -62, Math.toRadians(90))
                .build();

        thirdDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        secondIntake.getFinalPose().toPoint(),
                        //new Point(16, -48),
                        depositEndPose.plus(new Pose(-11, 0, 0)).toPoint()

                )
                .addFinalPose(depositEndPose.plus(new Pose(-11, 0, 0))) // this might need to be at a lower angle
                .build();

        thirdIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(0.6,
                        new Point(0, -28),
                        new Point(0, -54),
                        new Point(36, -37),
                        new Point(36, -62)
                )
//                .addFinalSpeed(0.3)
                .addFinalPose(36, -62, Math.toRadians(90))
                .build();

        forthDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(1,
                        thirdIntake.getFinalPose().toPoint(),
                        //new Point(16, -48),
                       depositEndPose.plus(new Pose(-13, 0, 0)).toPoint()

                )
                .addFinalPose(depositEndPose.plus(new Pose(-13, 0, 0))) // this might need to be at a lower angle
                .build();

        forthIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(0.6,
                        new Point(0, -28),
                        new Point(0, -54),
                        new Point(36, -37),
                        new Point(36, -62)
                )
//                .addFinalSpeed(0.3)
                .addFinalPose(36, -62, Math.toRadians(90))
                .build();
        fifthDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(1,
                        forthIntake.getFinalPose().toPoint(),
                        //new Point(16, -48),
                        depositEndPose.plus(new Pose(-15, 0, 0)).toPoint()

                )
                .addFinalPose(depositEndPose.plus(new Pose(-15, 0, 0))) // this might need to be at a lower angle
                .build();
        fifthIntake = new TrajectoryBuilder() // spline heading
                .addBezierSegment(0.6,
                        new Point(0, -28),
                        new Point(0, -54),
                        new Point(36, -37),
                        new Point(36, -62)
                )
//                .addFinalSpeed(0.3)
                .addFinalPose(36, -62, Math.toRadians(90))
                .build();
        sixthDeposit = new TrajectoryBuilder() // spline heading
                .addBezierSegment(1,
                        fifthIntake.getFinalPose().toPoint(),
                        //new Point(16, -48),
                        depositEndPose.plus(new Pose(-17, 0, 0)).toPoint()

                )
                .addFinalPose(depositEndPose.plus(new Pose(-17, 0, 0))) // this might need to be at a lower angle
                .build();
        parkTrajectory = new TrajectoryBuilder() // spline heading
                .addBezierSegment(
                        new Point(9, -38),
                        new Point(46, -60)

                )
                .addFinalPose(46, -60, Math.toRadians(0))
                .build();
    }

    public void generatePreloadTrajectory(Pose samplePose)
    {
        if (samplePose == null) return;
        double cameraXOffset = -5;
        double finalXPose = 7.2 + samplePose.getX() + cameraXOffset;
        Vector offSetVector = new Vector(-2.5, 2); // as center of rotation of the robot is not the camera
        double finalHeading = Math.toRadians(90) - samplePose.getHeading();
        offSetVector = offSetVector.rotated(-finalHeading);
        preloadTrajectory = new TrajectoryBuilder() // spline heading
                .addBezierSegment(1,
                        new Point(7.2, -62.5),
                        new Point( finalXPose - offSetVector.getX(),
                                -30 + offSetVector.getY())
                )
                .addFinalSpeed(0.7)
                .addFinalPose(finalXPose, -30, finalHeading) // limelight heading is inverted lol
                .build();
    }
}
