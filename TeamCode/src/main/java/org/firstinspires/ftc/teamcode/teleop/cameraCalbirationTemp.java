package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
@TeleOp
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
                if(detection.ftcPose != null){
                    double x = detection.ftcPose.x; // mm left/right
                    double y = detection.ftcPose.y; // mm up/down need to be adjusted add couple of inches
                    double z = detection.ftcPose.z; // mm forward (distance in front of camera)
                    double pitch = detection.ftcPose.pitch;


                    // 3D straight-line distance
                    double distance = Math.sqrt(x*x + y*y + z*z)/1000;

                    telemetry.addData("Tag ID: ", detection.id);
                    telemetry.addData("3D Distance (m): ", distance);
                    telemetry.addData("Distance X (m): ", x/1000);
                    telemetry.addData("Distance Y (m): ", y/1000);
                    telemetry.addData("Distance Z (m): ", z/1000);
                    telemetry.addData("Pitch in degrees (Rounded): ", Math.round(pitch));
                }
            }
            telemetry.update();
        }
    }

}
