package org.firstinspires.ftc.teamcode.lab;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ColorBallDetectionPipeline extends OpenCvPipeline {

    public static class BallData {
        public Point center;
        public double radius;
        public String color;   // "purple" or "yellow"
        public String region;  // "Left", "Middle", or "Right"
    }

    public List<BallData> detectedBalls = new ArrayList<>();

    // --- HSV color thresholds (adjust to your environment) ---
    private final Scalar lowerYellow = new Scalar(20, 100, 100);
    private final Scalar upperYellow = new Scalar(40, 255, 255);

    private final Scalar lowerPurple = new Scalar(130, 50, 50);
    private final Scalar upperPurple = new Scalar(160, 255, 255);

    private final Mat hsv = new Mat();
    private final Mat maskYellow = new Mat();
    private final Mat maskPurple = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, lowerYellow, upperYellow, maskYellow);
        Core.inRange(hsv, lowerPurple, upperPurple, maskPurple);

        detectedBalls.clear();

        detectBallsOfColor(maskYellow, input, "yellow", new Scalar(255, 255, 0));
        detectBallsOfColor(maskPurple, input, "purple", new Scalar(255, 0, 255));

        // Draw region lines
        int width = input.width();
        Imgproc.line(input, new Point(width / 3.0, 0), new Point(width / 3.0, input.height()), new Scalar(255, 255, 255), 2);
        Imgproc.line(input, new Point(2 * width / 3.0, 0), new Point(2 * width / 3.0, input.height()), new Scalar(255, 255, 255), 2);

        return input;
    }

    private void detectBallsOfColor(Mat mask, Mat input, String colorName, Scalar drawColor) {
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        int width = input.width();
        int regionWidth = width / 3;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area < 500) continue; // ignore small objects/noise

            Point center = new Point();
            float[] radius = new float[1];
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            Imgproc.minEnclosingCircle(contour2f, center, radius);

            BallData ball = new BallData();
            ball.center = center;
            ball.radius = radius[0];
            ball.color = colorName;

            // Determine region (Left, Middle, Right)
            if (center.x < regionWidth) {
                ball.region = "Left";
            } else if (center.x < 2 * regionWidth) {
                ball.region = "Middle";
            } else {
                ball.region = "Right";
            }

            detectedBalls.add(ball);

            // Draw circle and labels on screen
            Imgproc.circle(input, center, (int) radius[0], drawColor, 2);
            Imgproc.putText(input, colorName + " - " + ball.region,
                    new Point(center.x - 40, center.y - 10),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, drawColor, 2);
        }
    }
}
