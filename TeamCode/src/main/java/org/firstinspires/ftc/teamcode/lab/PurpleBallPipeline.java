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

    private final Mat kernel =
            Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5));

    private Mat roiMask = new Mat();

    // --- Confidence values (temporal smoothed) ---
    private double lastConfidencePurple = 0;
    private double lastConfidenceGreen = 0;
    private static final double SMOOTHING = 0.35;

    // --- Expected area values for confidence ---
    private static final double AREA_IDEAL = 30000;    // Tune for distance
    private static final double AREA_TOLERANCE = 20000;

    // --- Performance settings ---
    private boolean fastMode = true;
    private static final int BLUR_SIZE = 5;

    private long lastTime = 0;
    private double fps = 0;

    public static class BallData {
        public Point center;
        public float radius;
        public String region;
        public double area;
        public String color;

        public double confidence; // NEW FIELD
    }

    public final List<BallData> detectedBalls = new ArrayList<>();
    public final List<BallData> greenBalls = new ArrayList<>();

    private int frameWidth = 0;
    private int frameHeight = 0;

    private double splitFraction = 0.5;

    // HSV ranges
    private static final Scalar LOWER_PURPLE = new Scalar(100, 80, 50);
    private static final Scalar UPPER_PURPLE = new Scalar(160, 255, 255);

    private static final Scalar LOWER_GREEN = new Scalar(30, 60, 50);
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

    public void setFastMode(boolean enable) {
        fastMode = enable;
    }

    @Override
    public Mat processFrame(Mat input) {

        // ---------- FPS CALCULATION ----------
        long now = System.nanoTime();
        if (lastTime != 0) fps = 1e9 / (now - lastTime);
        lastTime = now;

        detectedBalls.clear();
        greenBalls.clear();

        frameWidth = input.width();
        frameHeight = input.height();

        input.copyTo(output);

        // Convert to HSV
        Imgproc.cvtColor(input, matHSV, Imgproc.COLOR_RGB2HSV);

        // ----------- PURPLE DETECTION -----------
        Core.inRange(matHSV, LOWER_PURPLE, UPPER_PURPLE, mask);

        if (fastMode) {
            Imgproc.morphologyEx(mask, morphed, Imgproc.MORPH_OPEN, kernel);
            Imgproc.GaussianBlur(morphed, blurred, new Size(BLUR_SIZE, BLUR_SIZE), 0);
        } else {
            Imgproc.morphologyEx(mask, morphed, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(morphed, morphed, Imgproc.MORPH_CLOSE, kernel);
            Imgproc.GaussianBlur(morphed, blurred, new Size(7, 7), 0);
        }
        findBallsFromMask(blurred, "Purple", detectedBalls, output);

        // ----------- GREEN DETECTION -----------
        Core.inRange(matHSV, LOWER_GREEN, UPPER_GREEN, mask);

        if (fastMode) {
            Imgproc.morphologyEx(mask, morphed, Imgproc.MORPH_OPEN, kernel);
            Imgproc.GaussianBlur(morphed, blurred, new Size(BLUR_SIZE, BLUR_SIZE), 0);
        } else {
            Imgproc.morphologyEx(mask, morphed, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(morphed, morphed, Imgproc.MORPH_CLOSE, kernel);
            Imgproc.GaussianBlur(morphed, blurred, new Size(7, 7), 0);
        }
        findBallsFromMask(blurred, "Green", greenBalls, output);

        // Split line
        Imgproc.line(output,
                new Point(frameWidth * splitFraction, 0),
                new Point(frameWidth * splitFraction, frameHeight),
                new Scalar(255, 255, 255), 2);

        // FPS draw
        Imgproc.putText(output,
                String.format("FPS: %.1f", fps),
                new Point(20, 40),
                Imgproc.FONT_HERSHEY_SIMPLEX, 1,
                new Scalar(255, 255, 255), 2);

        return output;
    }

    private void findBallsFromMask(Mat maskInput, String color,
                                   List<BallData> ballList, Mat output) {

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(maskInput, contours, hierarchy,
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {

            if (contour == null || contour.empty()) continue;

            Rect boundingBox = Imgproc.boundingRect(contour);
            if (boundingBox.width <= 0 || boundingBox.height <= 0) continue;

            Point center =
                    new Point(boundingBox.x + boundingBox.width / 2.0,
                            boundingBox.y + boundingBox.height / 2.0);

            float radius = Math.max(boundingBox.width, boundingBox.height) / 2.0f;
            if (radius < MIN_RADIUS || radius > MAX_RADIUS) continue;

            double area = Imgproc.contourArea(contour);
            if (area < 50) continue;

            // ----- CIRCULARITY -----
            double perimeter = Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true);
            double circularity = 4 * Math.PI * area / (perimeter * perimeter + 1e-6);

            // ----- MASK DENSITY -----
            Mat roiBall = maskInput.submat(boundingBox);
            double whitePixels = Core.countNonZero(roiBall);
            double totalPixels = roiBall.rows() * roiBall.cols();
            roiBall.release();

            double densityScore = Math.min(1.0, whitePixels / totalPixels);
            double areaScore =
                    1.0 - Math.min(1.0, Math.abs(area - AREA_IDEAL) / AREA_TOLERANCE);
            double circularityScore = Math.min(1.0, circularity / 0.9);

            double confidenceRaw =
                    (0.40 * areaScore) +
                            (0.35 * circularityScore) +
                            (0.25 * densityScore);

            // Temporal smoothing
            double confidence;
            if (color.equals("Purple")) {
                lastConfidencePurple =
                        lastConfidencePurple * (1 - SMOOTHING) + confidenceRaw * SMOOTHING;
                confidence = lastConfidencePurple;
            } else {
                lastConfidenceGreen =
                        lastConfidenceGreen * (1 - SMOOTHING) + confidenceRaw * SMOOTHING;
                confidence = lastConfidenceGreen;
            }

            // Remove duplicates
            boolean duplicate = false;
            for (BallData b : ballList) {
                double dx = b.center.x - center.x;
                double dy = b.center.y - center.y;
                if (Math.sqrt(dx * dx + dy * dy) < MIN_CENTER_DISTANCE) {
                    duplicate = true;
                    break;
                }
            }
            if (duplicate) continue;

            // ----- SAVE BALL -----
            BallData ball = new BallData();
            ball.center = center;
            ball.radius = radius;
            ball.area = area;
            ball.color = color;
            ball.region = (center.x < frameWidth * splitFraction) ? "Left" : "Right";
            ball.confidence = confidence;

            ballList.add(ball);

            // Draw
            Imgproc.circle(output, center, (int) radius,
                    color.equals("Purple") ? new Scalar(255, 0, 255) : new Scalar(0, 255, 0),
                    3);

            Imgproc.putText(output,
                    String.format("%s C: %.2f", color, confidence),
                    new Point(center.x - radius, center.y - radius - 5),
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    new Scalar(255, 255, 255),
                    2);

        }

        // Release contours
        for (MatOfPoint contour : contours) contour.release();
    }

    @Override
    public void onViewportTapped() {
        matHSV.release();
        mask.release();
        hierarchy.release();
        morphed.release();
        blurred.release();
        kernel.release();
        output.release();
    }
}
