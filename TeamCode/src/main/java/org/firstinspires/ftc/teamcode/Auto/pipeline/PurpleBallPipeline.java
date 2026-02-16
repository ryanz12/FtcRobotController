package org.firstinspires.ftc.teamcode.Auto.pipeline;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;

public class PurpleBallPipeline extends OpenCvPipeline {

    private static final Scalar LOWER_PURPLE = new Scalar(120, 50, 40);
    private static final Scalar UPPER_PURPLE = new Scalar(160, 255, 255);

    // --- AREA TUNING ---
    private static final double MIN_AREA = 1000;

    // Set this to the area of ONE typical purple ball
    private static final double SINGLE_BALL_AREA = 32000;

    private int ballCount = 0;
    private final Mat hsv = new Mat();
    private final Mat mask = new Mat();
    private final Mat morphed = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        ballCount = 0;
        StringBuilder areasDetected = new StringBuilder("Areas: ");

        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, LOWER_PURPLE, UPPER_PURPLE, mask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5));
        Imgproc.morphologyEx(mask, morphed, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(morphed, morphed, Imgproc.MORPH_CLOSE, kernel);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(morphed, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);

            if (area < MIN_AREA) continue;

            areasDetected.append((int)area).append(" | ");

            // --- MULTI-BALL DETECTION LOGIC ---
            int ballsInThisBlob;

            if (area >= SINGLE_BALL_AREA * 2.7) {
                // Roughly 3x the area
                ballsInThisBlob = 3;
            } else if (area >= SINGLE_BALL_AREA * 1.6) {
                // Roughly 2x the area
                ballsInThisBlob = 2;
            } else {
                // Just 1 ball
                ballsInThisBlob = 1;
            }

            ballCount += ballsInThisBlob;

            // Visual Feedback
            Imgproc.drawContours(input, List.of(contour), -1, new Scalar(0, 255, 0), 2);

            Rect rect = Imgproc.boundingRect(contour);
            String label = (ballsInThisBlob > 1) ? "CLUMP x" + ballsInThisBlob : "BALL";
            Imgproc.putText(input, label, new Point(rect.x, rect.y - 5),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 0), 2);
        }

        // --- SCREEN TELEMETRY ---
        Imgproc.putText(input, "Total Count: " + ballCount, new Point(20, 40),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(0, 255, 0), 2);

        Imgproc.putText(input, areasDetected.toString(), new Point(20, 80),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 1);

        return input;
    }

    public int getBallCount() { return ballCount; }
}