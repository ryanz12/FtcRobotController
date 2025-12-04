package org.firstinspires.ftc.teamcode.lab;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.openftc.easyopencv.*;
import org.firstinspires.ftc.teamcode.subsystem.BeltSubsystem;

import java.util.ArrayList;
import java.util.List;
@TeleOp
public class PurpleBallDetectionOpMode extends LinearOpMode {

    private OpenCvWebcam webcam;
    private PurpleBallPipeline pipeline;
    private BeltSubsystem beltSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private DistanceSensor distanceSensor;

    private boolean firstBallAdjusted = false;

    private static final double FIRST_BALL_X_TARGET = 530.0;
    private static final double FIRST_BALL_Y_TARGET = 330.0;

    private double lastKnownX = 0;
    private double lastKnownY = 0;
    private long lastDetectionTime = 0;
    private static final long DETECTION_TIMEOUT = 1500;

    private int distanceUnder18Count = 0;
    private boolean ballPreviouslyDetected = false;

    // --- B + Y combo tracking ---
    private boolean bPreviouslyPressed = false;
    private boolean yPreviouslyPressed = false;
    private boolean bPressedOnce = false;

    // --- Timer for delayed reset ---
    private boolean resetTimerActive = false;
    private long resetStartTime = 0;
    private static final long RESET_DELAY_MS = 3000; // 3 seconds

    @Override
    public void runOpMode() {

        beltSubsystem = new BeltSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new PurpleBallPipeline(telemetry);
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera error", errorCode);
                telemetry.update();
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            boolean rbPressed = gamepad2.right_bumper;

            // --- System logic only runs when RB is held ---
            if (rbPressed) {
                double distanceCm = distanceSensor.getDistance(DistanceUnit.CM);
                boolean ballDetected = distanceCm < 18.0;

                if (ballDetected && !ballPreviouslyDetected) {
                    distanceUnder18Count++;
                }
                ballPreviouslyDetected = ballDetected;

                // --- Ball detection via pipeline ---
                List<PurpleBallPipeline.BallData> balls = new ArrayList<>();
                synchronized (pipeline) {
                    balls.addAll(pipeline.detectedBalls);
                    balls.addAll(pipeline.greenBalls);
                }

                PurpleBallPipeline.BallData firstBall = null;
                for (PurpleBallPipeline.BallData ball : balls) {
                    if (ball != null && ball.center != null && "Left".equals(ball.region)) {
                        if (firstBall == null || ball.center.x < firstBall.center.x) {
                            firstBall = ball;
                        }
                    }
                }

                long currentTime = System.currentTimeMillis();

                if (firstBall != null) {
                    lastKnownX = firstBall.center.x;
                    lastKnownY = firstBall.center.y;
                    lastDetectionTime = currentTime;
                }

                // --- Intake control ---
                if (distanceUnder18Count <= 2) {
                    intakeSubsystem.intake();
                }

                // --- Belt control ---
                if (distanceUnder18Count > 2) {
                    beltSubsystem.stop();
                    intakeSubsystem.stop();
                    telemetry.addLine("Stopping belt: 2 balls detected by distance sensor");
                } else {
                    if (!firstBallAdjusted) {
                        if ((lastKnownX < FIRST_BALL_X_TARGET || lastKnownY < FIRST_BALL_Y_TARGET) &&
                                (currentTime - lastDetectionTime <= DETECTION_TIMEOUT)) {
                            beltSubsystem.move_belt(BeltSubsystem.Direction.DTR);
//                            telemetry.addLine(String.format(
//                                    "Adjusting first ball: X=%.0f (target>%.0f), Y=%.0f (target>%.0f)",
//                                    lastKnownX, FIRST_BALL_X_TARGET, lastKnownY, FIRST_BALL_Y_TARGET));
                        } else {
                            beltSubsystem.stop();
                            if (lastKnownX >= FIRST_BALL_X_TARGET && lastKnownY >= FIRST_BALL_Y_TARGET) {
                                firstBallAdjusted = true;
//                                telemetry.addLine(String.format(
//                                        "First ball reached X=%.0f and Y=%.0f – stopped belt.",
//                                        lastKnownX, lastKnownY));
                            } else {
                                telemetry.addLine("Timeout reached – stopping belt.");
                            }
                        }
                    } else {
                        beltSubsystem.stop();
//                        telemetry.addLine("First ball adjustment complete – holding belt still.");
                    }
                }

//                telemetry.addLine("RB held – logic running");
//                telemetry.addData("First Ball Adjusted", firstBallAdjusted);
//                telemetry.addData("Last Known Position", "X: %.0f, Y: %.0f", lastKnownX, lastKnownY);
//                telemetry.addData("Detected Balls (Purple + Green)", balls.size());
//                telemetry.addData("Distance (cm)", distanceCm);
//                telemetry.addData("Balls Counted by Distance Sensor", distanceUnder18Count);

            } else {
                // --- RB not held: stop everything ---
                beltSubsystem.stop();
                intakeSubsystem.stop();
//                telemetry.addLine("RB not held – systems paused");
            }

            // --- B → Y combo reset logic (independent of RB) ---
            boolean bPressed = gamepad2.b;
            boolean yPressed = gamepad2.y;

            if (bPressed && !bPreviouslyPressed) {
                bPressedOnce = true; // B pressed
            }

            if (bPressedOnce && yPressed && !yPreviouslyPressed && !resetTimerActive) {
                resetTimerActive = true;
                resetStartTime = System.currentTimeMillis();
//                telemetry.addLine("B + Y detected – resetting in 3 seconds...");
            }

            if (resetTimerActive && (System.currentTimeMillis() - resetStartTime >= RESET_DELAY_MS)) {
                distanceUnder18Count = 0;
                resetTimerActive = false;
                bPressedOnce = false;
//                telemetry.addLine("DistanceUnder18Count reset to 0 after 3-second delay.");
            }

            bPreviouslyPressed = bPressed;
            yPreviouslyPressed = yPressed;

//            telemetry.addData("Reset Timer Active", resetTimerActive);
            telemetry.update();
        }

        beltSubsystem.stop();
    }
}
