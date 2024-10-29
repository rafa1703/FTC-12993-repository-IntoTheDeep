package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware.S;

import org.firstinspires.ftc.teamcode.gvf.trajectories.BezierCurveTrajectorySegment;
import org.firstinspires.ftc.teamcode.gvf.trajectories.Trajectory;
import org.firstinspires.ftc.teamcode.gvf.trajectories.TrajectoryBuilder;
import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.opencv.core.Point;

public class Paths
{
    public Trajectory depositFarTrajectory, depositToSamplesFarTrajectory, samplesToHPFarTrajectory, hpToParkFarTrajectory;
    public Trajectory depositCloseTrajectory, depositToSamplesCloseTrajectory, samplesToBucketTrajectory, bucketToParkTrajectory;
    public Trajectory firstIntakeTrajectory, secondIntakeTrajectory, thirdIntakeTrajectory,
            firstDepositTrajectory, secondDepositTrajectory, thirdDepositTrajectory, parkTrajectory;
    public Paths()
    {
        /*depositFarTrajectory = new TrajectoryBuilder(new Pose(9.5, -62.3 * S, Math.toRadians(90)))
                .addSegment(new BezierCurveTrajectorySegment(
                        new Point[]{
                                new Point(9.5, -62.3 * S),
                                new Point(8.5, -36 * S)
                        }
                ))
                .build();
        depositToSamplesFarTrajectory = new TrajectoryBuilder(depositFarTrajectory.getFinalPose())
                .addSegment(new BezierCurveTrajectorySegment(
                        new Point[]{
                                depositFarTrajectory.getFinalPose().toPoint(),
                                new Point(32, -32 * S),
                                new Point(42, -10 * S)
                                // new Point(6, -36)
                        }
                ))
                .addFinalPose(new Pose(42, -10 * S, Math.toRadians(180)))
                .build();
        samplesToHPFarTrajectory = new TrajectoryBuilder(depositToSamplesFarTrajectory.getFinalPose())
                .addSegment(new BezierCurveTrajectorySegment(
                        new Point[]{
                                depositToSamplesFarTrajectory.getFinalPose().toPoint(),
                                new Point(55, -37 * S),
                                new Point(54, -54 * S)
                                // new Point(6, -36)
                        }
                ))
                .addFinalPose(new Pose(54, -54 * S, Math.toRadians(180)))
                .build();
        hpToParkFarTrajectory = new TrajectoryBuilder(samplesToHPFarTrajectory.getFinalPose())
                .addSegment(new BezierCurveTrajectorySegment(
                        new Point[]{
                                samplesToHPFarTrajectory.getFinalPose().toPoint(),
                                new Point(32, -10 * S)
                        }
                ))
                .addFinalPose(depositToSamplesFarTrajectory.getFinalPose())
                .build();

        depositCloseTrajectory = new TrajectoryBuilder(new Pose(9.5, -62.3 * S, Math.toRadians(90)))
                .addSegment(new BezierCurveTrajectorySegment(
                        new Point[]{
                                new Point(9.5, -62.3 * S),
                                new Point(8.5, -36 * S)
                        }
                ))
                .build();
        depositToSamplesCloseTrajectory = new TrajectoryBuilder(depositFarTrajectory.getFinalPose())
                .addSegment(new BezierCurveTrajectorySegment(
                        new Point[]{
                                depositFarTrajectory.getFinalPose().toPoint(),
                                new Point(32, -32 * S),
                                new Point(42, -10 * S)
                                // new Point(6, -36)
                        }
                ))
                .addFinalPose(new Pose(42, -10 * S, Math.toRadians(180)))
                .build();
        samplesToBucketTrajectory = new TrajectoryBuilder(depositToSamplesFarTrajectory.getFinalPose())
                .addSegment(new BezierCurveTrajectorySegment(
                        new Point[]{
                                depositToSamplesFarTrajectory.getFinalPose().toPoint(),
                                new Point(55, -37 * S),
                                new Point(54, -54 * S)
                                // new Point(6, -36)
                        }
                ))
                .addFinalPose(new Pose(54, -54 * S, Math.toRadians(180)))
                .build();
        bucketToParkTrajectory = new TrajectoryBuilder(samplesToHPFarTrajectory.getFinalPose())
                .addSegment(new BezierCurveTrajectorySegment(
                        new Point[]{
                                samplesToHPFarTrajectory.getFinalPose().toPoint(),
                                new Point(32, -10 * S)
                        }
                ))
                .addFinalPose(depositToSamplesFarTrajectory.getFinalPose())
                .build();*/
        firstIntakeTrajectory = new TrajectoryBuilder(new Pose(9.5, -62.3, Math.toRadians(90))) //SplineHeading
                .addSegment(new BezierCurveTrajectorySegment(
                        new Point[]{
                                new Point(-9.5, -62.3),
                                new Point(-42.1, -31.8)
                        }
                ))
                .addFinalPose(new Pose(-42.1, -31.8, Math.toRadians(145)))
                .build();

        firstDepositTrajectory = new TrajectoryBuilder(firstIntakeTrajectory.getFinalPose()) //SplineHeading
                .addSegment(new BezierCurveTrajectorySegment(
                        new Point[]{
                                firstIntakeTrajectory.getFinalPose().toPoint(),
                                new Point(-50, -59),
                                new Point(-58.8, -56.7),
                                // new Point(6, -36)
                        }
                ))
                .addFinalPose(new Pose(-58.8, -56.7, Math.toRadians(45)))
                .build();

        secondIntakeTrajectory = new TrajectoryBuilder(firstDepositTrajectory.getFinalPose()) //SplineHeading
                .addSegment(new BezierCurveTrajectorySegment(
                        new Point[]{
                                firstDepositTrajectory.getFinalPose().toPoint(),
                                new Point(-65, -45),
                                new Point(-60, -34)
                                // new Point(6, -36)
                        }
                ))
                .addFinalPose(new Pose(-60, -34, Math.toRadians(90)))
                .build();

        secondDepositTrajectory = new TrajectoryBuilder(secondIntakeTrajectory.getFinalPose()) // Tangent(reverse)
                .addSegment(new BezierCurveTrajectorySegment(
                        new Point[]{
                                secondIntakeTrajectory.getFinalPose().toPoint(),
                                new Point(-63, -56.7)
                        }
                ))
                .addFinalPose(new Pose(-63, -56.7, Math.toRadians(90)))
                .build();
        parkTrajectory = new TrajectoryBuilder(new Pose(-63, -56.7, Math.toRadians(90))) //SplineHeading
                .addSegment(new BezierCurveTrajectorySegment(
                        new Point[]{
                                new Point(-63, -56.7),
                                new Point(-32, -10)
                        }
                ))
                .addFinalPose(new Pose(-32, -10, Math.toRadians(0)))
                .build();
    }
}
