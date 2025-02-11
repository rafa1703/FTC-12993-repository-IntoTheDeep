package org.firstinspires.ftc.teamcode.system.visiontest;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class SamplePipeline extends OpenCvPipeline
{
    Telemetry telemetry;
    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6));
    public SamplePipeline(Telemetry telemetry)
    {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input)
    {
        Mat cbMat = new Mat();
        Mat thresholdMat = new Mat();
        Mat morphedThreshold = new Mat();
        Mat contourMat = new Mat();
        Mat edges = new Mat();

        // A list we'll be using to store the contours we find
        ArrayList<MatOfPoint> contoursList = new ArrayList<>();

        // Convert the input image to YCrCb color space, then extract the Cb channel
        Imgproc.cvtColor(input, cbMat, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(cbMat, cbMat, 2);

        // Threshold the Cb channel to form a mask, then run some noise reduction
        Imgproc.threshold(cbMat, thresholdMat, 150, 255, Imgproc.THRESH_BINARY);
        //morphMask(thresholdMat, morphedThreshold);

        //Imgproc.Canny(thresholdMat, edges, 0, 10);
        // Ok, now actually look for the contours! We only look for external contours.
        Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        input.copyTo(contourMat);
        Imgproc.drawContours(contourMat, contoursList, -1, new Scalar(255, 255, 255), 2, 8);

        for (MatOfPoint contour : contoursList)
        {
            analyzeContour(contour, input);
        }

        //cbMat.release();
        //thresholdMat.release();
        morphedThreshold.release();
        //contourMat.release();
        //edges.release();
        return contourMat;
    }
    void analyzeContour(MatOfPoint contour, Mat input)
    {
        // Transform the contour to a different format
        Point[] points = contour.toArray();
        MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

        // Do a rect fit to the contour, and draw it on the screen
        RotatedRect rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);
        drawRotatedRect(rotatedRectFitToContour, input);
        double rotRectAngle = rotatedRectFitToContour.angle;
        if (rotatedRectFitToContour.size.width < rotatedRectFitToContour.size.height)
        {
            rotRectAngle += 90;
        }
        telemetry.addData("Angle", rotRectAngle);
        telemetry.addData("Is sample straight", rotRectAngle >= 80 && rotRectAngle <= 100 ? "True" : "False");
        telemetry.update();
    }
    static void drawRotatedRect(RotatedRect rect, Mat drawOn)
    {
        /*
         * Draws a rotated rect by drawing each of the 4 lines individually
         */

        Point[] points = new Point[4];
        rect.points(points);

        for(int i = 0; i < 4; ++i)
        {
            Imgproc.line(drawOn, points[i], points[(i+1)%4], new Scalar(255, 0, 0), 2);
        }
    }
    private void morphMask(Mat input, Mat output)
    {
        /*
         * Apply some erosion and dilation for noise reduction
         */

        Imgproc.erode(input, output, erodeElement);
        Imgproc.erode(output, output, erodeElement);

        Imgproc.dilate(output, output, dilateElement);
        Imgproc.dilate(output, output, dilateElement);
    }
}
