package org.firstinspires.ftc.teamcode.lab;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;
public class AprilTagDistance extends LinearOpMode{
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(50);

        // 1) prepare the AprilTag processor (easy defaults)
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        // 2) build VisionPortal using the webcam named "Webcam 2"
        //    (this is the hardwareMap name you set in the Robot Configuration)
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 2");

        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(aprilTagProcessor)
                // you can tweak resolution / stream format here if needed
                .enableLiveView(false) // false recommended for competition to save bandwidth
                .build();

        // wait for camera to start streaming (optional, useful for debugging)
        while (!isStopRequested() && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("camera", "waiting to stream (%s)", visionPortal.getCameraState());
            telemetry.update();
            sleep(50);
        }

        telemetry.addLine("Ready - press Play");
        telemetry.update();
        waitForStart();

        // main loop: read detections and show pose / distance
        while (!isStopRequested() && opModeIsActive()) {
            List<AprilTagDetection> dets = aprilTagProcessor.getDetections();

            if (dets != null && dets.size() > 0) {
                // process all detections (here we show the first one, but loop if you want)
                AprilTagDetection d = dets.get(0);

                telemetry.addData("tagId", d.id);

                // ftcPose may be null (pose estimation can fail) — check before use
                if (d.ftcPose != null) {
                    double x = d.ftcPose.x;   // left/right (metres) relative to camera
                    double y = d.ftcPose.y;   // forward/back (metres) relative to camera
                    double z = d.ftcPose.z;   // up/down (metres) relative to camera
                    double roll = d.ftcPose.roll;
                    double pitch = d.ftcPose.pitch;
                    double yaw = d.ftcPose.yaw;

                    // simple Euclidean range (distance) to tag center (metres)
                    double range = Math.sqrt(x*x + y*y + z*z);

                    telemetry.addData("pose", "x=%.3f y=%.3f z=%.3f", x, y, z);
                    telemetry.addData("angles", "r=%.2f p=%.2f y=%.2f", roll, pitch, yaw);
                    telemetry.addData("range", "%.3f m", range);
                } else {
                    telemetry.addLine("pose: <not available> (ftcPose == null)");
                }
            } else {
                telemetry.addLine("no tags");
            }

            telemetry.update();
            sleep(50);
        }

        // cleanup
        visionPortal.stopStreaming();
        visionPortal.close();
    }

}
