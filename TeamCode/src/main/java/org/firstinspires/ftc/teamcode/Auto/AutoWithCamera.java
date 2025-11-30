package org.firstinspires.ftc.teamcode.Auto;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.lab.PurpleBallPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Autonomous
public class AutoWithCamera extends LinearOpMode {

    private CRServo rampOne, rampTwo;
    private OpenCvCamera webcam;
    private PurpleBallPipeline pipeline;

    private class Ramp {
        private CRServo rampServoOne, rampServoTwo;

        public Ramp() {
            rampServoOne = hardwareMap.get(CRServo.class, "beltServoOne");
            rampServoTwo = hardwareMap.get(CRServo.class, "beltServoTwo");
        }

        public Action rampUpUntilNoPurple(PurpleBallPipeline pipeline) {
            return new Action() {
                @Override
                public boolean run(@NonNull com.acmerobotics.dashboard.telemetry.TelemetryPacket packet) {
                    if (!pipeline.detectedBalls.isEmpty()) {
                        rampServoOne.setPower(1);
                        rampServoTwo.setPower(-1);
                        packet.put("Purple Count", pipeline.detectedBalls.size());
                        return true;
                    } else {
                        rampServoOne.setPower(0);
                        rampServoTwo.setPower(0);
                        return false;
                    }
                }
            };
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        webcam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        pipeline = new PurpleBallPipeline(telemetry);
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        Ramp ramp = new Ramp();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(
                ramp.rampUpUntilNoPurple(pipeline)
        ));
    }
}