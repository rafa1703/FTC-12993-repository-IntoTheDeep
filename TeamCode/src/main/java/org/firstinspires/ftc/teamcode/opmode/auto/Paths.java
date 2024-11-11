package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.system.hardware.robot.GeneralHardware.S;

import org.firstinspires.ftc.teamcode.gvf.trajectories.BezierCurveTrajectorySegment;
import org.firstinspires.ftc.teamcode.gvf.trajectories.Trajectory;
import org.firstinspires.ftc.teamcode.gvf.trajectories.TrajectoryBuilder;
import org.firstinspires.ftc.teamcode.gvf.utils.Pose;
import org.opencv.core.Point;

public class Paths
{

    public Trajectory firstIntakeTrajectory, secondIntakeTrajectory, preloadTrajectory,
            firstDepositTrajectory, secondDepositTrajectory, parkTrajectory, goBackAfterDrop;
    public Paths()
    {
        /*firstIntakeTrajectory = new TrajectoryBuilder(new Pose(9.5, -62.3, Math.toRadians(90))) //SplineHeading
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
                .build();*/

        preloadTrajectory = new TrajectoryBuilder() // splineHeading
                .addBezierSegment(
                        new Point(-3.5, -62.3),
                        new Point(-51.5,-56.5)

                )
                .addFinalPose(-51.5,-56.5, Math.toRadians(45))
                .build();
        firstIntakeTrajectory = new TrajectoryBuilder() // tangent
                .addBezierSegment(
                        preloadTrajectory.getFinalPose().toPoint(),
                        new Point(-45, -42),
                        new Point(-45, -47),
                        new Point(-46, -38),
                        new Point(-46, -33.8)
                )
                .build();
        firstDepositTrajectory = new TrajectoryBuilder() // tangent
                .addBezierSegment(
                        new Point(-44, -33.8),
                        new Point(-44, -38),
                        new Point(-43, -42),
                        preloadTrajectory.getFinalPose().toPoint()
                        )
                .addFinalPose(preloadTrajectory.getFinalPose())
                .build();

        secondIntakeTrajectory = new TrajectoryBuilder() // splineHeading
                .addBezierSegment(
                        new Point(-51.5 ,-56.5),
                        new Point(-53.9, -33.8)
                )
                .addFinalPose(-53.9, -33.8, Math.toRadians(90))
                .build();

        secondDepositTrajectory = new TrajectoryBuilder() // splineHeading
                .addBezierSegment(
                        secondIntakeTrajectory.getFinalPose().toPoint(),
                        preloadTrajectory.getFinalPose().toPoint()
                )
                .addFinalPose(preloadTrajectory.getFinalPose())
                .build();

        goBackAfterDrop = new TrajectoryBuilder()
                .addBezierSegment(
                        preloadTrajectory.getFinalPose().toPoint(),
                        new Point(-50, -50)
                )
                .build();

        parkTrajectory = new TrajectoryBuilder() // SplineHeading
                .addBezierSegment(
                        secondDepositTrajectory.getFinalPose().toPoint(),
                        new Point(-32, -11.5)
                )
                .addFinalPose(-32, -11.5, Math.toRadians(0))
                .build();
    }
}
