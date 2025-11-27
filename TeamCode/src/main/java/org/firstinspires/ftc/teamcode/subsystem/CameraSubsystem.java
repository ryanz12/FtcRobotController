package org.firstinspires.ftc.teamcode.subsystem;

import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Arrays;
import java.util.List;

public class CameraSubsystem extends SubsystemBase {
    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;

    public CameraSubsystem(HardwareMap hMap, Telemetry telemetry){

        aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hMap.get(WebcamName.class, "Webcam 2"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTag);

        visionPortal = builder.build();
    }

    public List<Double> getCameraData(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        if (!currentDetections.isEmpty()){
            AprilTagDetection detection = currentDetections.get(0);
            return Arrays.asList(detection.ftcPose.x, detection.ftcPose.range);
        }

        return null;
    }

    public void startStreaming(){
        visionPortal.setProcessorEnabled(aprilTag, true);
    }

    public VisionPortal.CameraState getCameraState(){
        return visionPortal.getCameraState();
    }

    public void stopStreaming(){
        visionPortal.setProcessorEnabled(aprilTag, false);
    }

}
