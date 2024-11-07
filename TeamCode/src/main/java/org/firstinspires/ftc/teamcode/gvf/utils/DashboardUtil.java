package org.firstinspires.ftc.teamcode.gvf.utils;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.gvf.trajectories.Trajectory;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.List;

public class DashboardUtil
{
    private static final double ROBOT_HEIGHT = 16;
    private static final double ROBOT_WIDTH = 14.5;


    public static void drawPoseHistory(Canvas canvas, List<Pose2d> poseHistory) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];
        for (int i = 0; i < poseHistory.size(); i++) {
            Pose2d pose = poseHistory.get(i);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawSampledPath(Canvas canvas, Trajectory path) {
        double[] xPoints = new double[(int) path.length()];
        double[] yPoints = new double[(int) path.length()];
        int i = 0;
        for (Point point: path.getFullCurve())
        {
            xPoints[i] = point.x;
            yPoints[i] = point.y;
            i++;
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawRobot(Canvas canvas, Pose2d pose) {
        canvas.setStroke("Red").strokeRect(pose.getX() - ROBOT_WIDTH / 2, pose.getY()- ROBOT_HEIGHT / 2, ROBOT_WIDTH, ROBOT_HEIGHT);
        Vector2d v = pose.headingVec().times(6);
        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
    }

    public static void drawRobot(Canvas canvas, Pose2d pose, boolean RR) {
        canvas.strokeCircle(pose.getX(), pose.getY(), 9);
        Vector2d v = pose.headingVec().times(9);
        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
    }
    public static void drawRobot(Canvas canvas, Pose2d pose, boolean RR, String color) {
        canvas.setStroke(color).strokeCircle(pose.getX(), pose.getY(), 9);
        Vector2d v = pose.headingVec().times(9);
        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
    }

}
