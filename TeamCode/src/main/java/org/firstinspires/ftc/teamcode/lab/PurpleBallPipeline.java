package org.firstinspires.ftc.teamcode.lab;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class PurpleBallPipeline extends OpenCvPipeline {

    private final Telemetry telemetry;

    // --- Preallocated Mats ---
    private final Mat matHSV = new Mat();
    private final Mat mask = new Mat();
    private final Mat hierarchy = new Mat();
    private final Mat morphed = new Mat();
    private final Mat blurred = new Mat();
    private final Mat output = new Mat();

    // Preallocated reusable kernel
    private final Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5));

    public static class BallData {
        public Point center;
        public float radius;
        public String region;
        public double area;
        public String color;
    }

    public final List<BallData> detectedBalls = new ArrayList<>();
    public final List<BallData> greenBalls = new ArrayList<>();

    private int frameWidth = 0;
    private int frameHeight = 0;

    private double splitFraction = 0.5;

    // HSV ranges
    private static final Scalar LOWER_PURPLE = new Scalar(125, 80, 50);
    private static final Scalar UPPER_PURPLE = new Scalar(160, 255, 255);

    private static final Scalar LOWER_GREEN = new Scalar(45, 60, 50);
    private static final Scalar UPPER_GREEN = new Scalar(90, 255, 255);

    private static final int MIN_RADIUS = 100;
    private static final int MAX_RADIUS = 300;
    private static final int MIN_CENTER_DISTANCE = 25;

    public PurpleBallPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void setSplitFraction(double fraction) {
        splitFraction = Math.max(2.0 / 3.0, Math.min(1.0, fraction));
    }

    @Override
    public Mat processFrame(Mat input) {
        detectedBalls.clear();
        greenBalls.clear();

        frameWidth = input.width();
        frameHeight = input.height();

        // Reuse preallocated output Mat
        input.copyTo(output);

        // Convert to HSV (reusing matHSV)
        Imgproc.cvtColor(input, matHSV, Imgproc.COLOR_RGB2HSV);

        // --- Purple Detection ---
        Core.inRange(matHSV, LOWER_PURPLE, UPPER_PURPLE, mask);
        Imgproc.morphologyEx(mask, morphed, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(morphed, morphed, Imgproc.MORPH_CLOSE, kernel);
        Imgproc.GaussianBlur(morphed, blurred, new Size(7, 7), 0);
        findBallsFromMask(blurred, "Purple", detectedBalls, output);

        // --- Green Detection ---
        Core.inRange(matHSV, LOWER_GREEN, UPPER_GREEN, mask);
        Imgproc.morphologyEx(mask, morphed, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(morphed, morphed, Imgproc.MORPH_CLOSE, kernel);
        Imgproc.GaussianBlur(morphed, blurred, new Size(7, 7), 0);
        findBallsFromMask(blurred, "Green", greenBalls, output);

        // Draw split line
        Imgproc.line(output,
                new Point(frameWidth * splitFraction, 0),
                new Point(frameWidth * splitFraction, frameHeight),
                new Scalar(255, 255, 255), 2);

        return output;
    }

    private void findBallsFromMask(Mat maskInput, String color, List<BallData> ballList, Mat output) {
        List<MatOfPoint> contours = new ArrayList<>();

        // Directly use maskInput (no clone!)
        Imgproc.findContours(maskInput, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        int count = 0;
        for (MatOfPoint contour : contours) {
            if (contour == null || contour.empty()) continue;

            Rect boundingBox = Imgproc.boundingRect(contour);
            if (boundingBox.width <= 0 || boundingBox.height <= 0) continue;

            Point center = new Point(boundingBox.x + boundingBox.width / 2.0, boundingBox.y + boundingBox.height / 2.0);
            float radius = Math.max(boundingBox.width, boundingBox.height) / 2.0f;
            if (radius < MIN_RADIUS || radius > MAX_RADIUS) continue;

            double area = Imgproc.contourArea(contour);

            boolean duplicate = false;
            for (BallData b : ballList) {
                if (b.center != null) {
                    double dx = b.center.x - center.x;
                    double dy = b.center.y - center.y;
                    if (Math.sqrt(dx * dx + dy * dy) < MIN_CENTER_DISTANCE) {
                        duplicate = true;
                        break;
                    }
                }
            }
            if (duplicate) continue;

            BallData ball = new BallData();
            ball.center = center;
            ball.radius = radius;
            ball.area = area;
            ball.region = (center.x < frameWidth * splitFraction) ? "Left" : "Right";
            ball.color = color;
            ballList.add(ball);

            Imgproc.circle(output, center, (int) radius,
                    color.equals("Purple") ? new Scalar(255, 0, 255) : new Scalar(0, 255, 0), 3);
            Imgproc.putText(output,
                    String.format("%s: %.0f (%.0f, %.0f)", color, area, center.x, center.y),
                    new Point(center.x - radius, center.y - radius - 5),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.5,
                    new Scalar(0, 255, 0), 2);
            count++;
        }

        // Release contours safely
        for (MatOfPoint contour : contours) {
            contour.release();
        }
    }

    @Override
    public void onViewportTapped() {
        // Release all preallocated Mats safely
        matHSV.release();
        mask.release();
        hierarchy.release();
        morphed.release();
        blurred.release();
        kernel.release();
        output.release();
    }
}
