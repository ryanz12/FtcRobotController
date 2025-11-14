package org.firstinspires.ftc.teamcode.lab;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

@TeleOp(name = "AprilTag ")
public class AprilTagDistance extends LinearOpMode {
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() throws InterruptedException {

        // 1) Create a tag library with the correct size (6.5 inches = 16.51 cm = 0.1651 m)
        double tagSizeMeters = 0.1651;
        AprilTagLibrary tagLibrary = new AprilTagLibrary.Builder()
                .addTag(
                        1,                // your tag ID
                        "MyTag",          // tag name (optional)
                        tagSizeMeters,    // tag size in meters
                        DistanceUnit.METER
                )
                .build();

        // 2) Create AprilTag processor with this library
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(tagLibrary)
                .setDrawAxes(true)
                .build();

        // 3) Build VisionPortal with webcam "Webcam 2"
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .addProcessor(aprilTag)
                .enableLiveView(false)
                .build();

        waitForStart();

        // 4) Main loop: detect tags and print x/y/z and 3D distance in cm
        while (opModeIsActive()) {
            List<AprilTagDetection> detections = aprilTag.getDetections();

            if (detections != null && !detections.isEmpty()) {
                for (AprilTagDetection detection : detections) {
                    if (detection.ftcPose != null) {
                        double xCm = detection.ftcPose.x * 100; // left/right
                        double yCm = detection.ftcPose.y * 100; // up/down
                        double zCm = detection.ftcPose.z * 100; // forward

                        double distanceCm = Math.sqrt(xCm*xCm + yCm*yCm + zCm*zCm);

                        telemetry.addData("Tag ID", detection.id);
                        telemetry.addData("X (cm)", "%.1f", xCm);
                        telemetry.addData("Y (cm)", "%.1f", yCm);
                        telemetry.addData("Forward (cm)", "%.1f", zCm);
                        telemetry.addData("3D Distance (cm)", "%.1f", distanceCm);
                    }
                }
            } else {
                telemetry.addLine("No tags detected");
            }

            telemetry.update();
        }

        // 5) Cleanup
        visionPortal.stopStreaming();
        visionPortal.close();
    }
}
