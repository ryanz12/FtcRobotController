package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
public class cameraCalbirationTemp extends LinearOpMode {
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() throws InterruptedException {
        // Create AprilTag processor
        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // Build VisionPortal with your webcam
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        waitForStart();

        while (opModeIsActive()) {
            for (AprilTagDetection detection : aprilTag.getDetections()) {
                double x = detection.ftcPose.x; // mm left/right
                double y = detection.ftcPose.y; // mm up/down
                double z = detection.ftcPose.z; // mm forward (distance in front of camera)

                // 3D straight-line distance
                double distance = Math.sqrt(x*x + y*y + z*z);

                telemetry.addData("Tag ID", detection.id);
                telemetry.addData("Forward Distance (mm)", z);
                telemetry.addData("3D Distance (mm)", distance);
            }
            telemetry.update();
        }
    }

}
