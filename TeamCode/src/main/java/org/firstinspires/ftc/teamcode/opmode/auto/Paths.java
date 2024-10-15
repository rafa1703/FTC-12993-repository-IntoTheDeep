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
    public Trajectory depositCloseTrajectory, depositToSamplesCloseTrajectory, samplesToBucketTrajectory, bucketToParkFarTrajectory;
    public Paths()
    {
        depositFarTrajectory = new TrajectoryBuilder(new Pose(9.5, -62.3 * S, Math.toRadians(90)))
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
        bucketToParkFarTrajectory = new TrajectoryBuilder(samplesToHPFarTrajectory.getFinalPose())
                .addSegment(new BezierCurveTrajectorySegment(
                        new Point[]{
                                samplesToHPFarTrajectory.getFinalPose().toPoint(),
                                new Point(32, -10 * S)
                        }
                ))
                .addFinalPose(depositToSamplesFarTrajectory.getFinalPose())
                .build();

    }
}
