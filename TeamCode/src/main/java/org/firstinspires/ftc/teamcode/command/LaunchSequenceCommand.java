package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lab.PurpleBallPipeline;
import org.firstinspires.ftc.teamcode.subsystem.BeltSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;


public class LaunchSequenceCommand extends CommandBase {

    private final Telemetry telemetry;
    private final BeltSubsystem beltSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final DistanceSensor distanceSensor;
    private final OpenCvWebcam webcam;
    private final PurpleBallPipeline pipeline;

    private final Supplier<Boolean> rbPressedSupplier;
    private final Supplier<Boolean> bPressedSupplier;
    private final Supplier<Boolean> yPressedSupplier;
    private final Supplier<Boolean> manualOverrideSupplier;

    private double lastKnownX = 0, lastKnownY = 0;
    private long lastDetectionTime = 0;
    private static final long DETECTION_TIMEOUT = 1500;
    private int amountOfBalls = 0;
    private boolean ballPreviouslyDetected = false;

    // Removed yPreviouslyPressed
    private boolean bPreviouslyPressed = false;
    private boolean waitingForY = false;

    private boolean pauseActive = false;
    private long pauseStartTime = 0;
    private static final long PAUSE_DURATION_MS = 3000;
    private boolean viewportPaused = false;
    private boolean waitingForFirstFrameAfterPause = false;
    private static final double FIRST_BALL_X_TARGET = 530.0;
    private static final double FIRST_BALL_Y_TARGET = 330.0;

    private enum State {
        WAITING_FOR_BALL,
        ADJUSTING_FIRST_BALL,
        FIRST_BALL_READY
    }

    private State state = State.WAITING_FOR_BALL;

    public LaunchSequenceCommand(
            BeltSubsystem belt, IntakeSubsystem intake,
            DistanceSensor distanceSensor, OpenCvWebcam webcam,
            PurpleBallPipeline pipeline,
            Supplier<Boolean> rbPressed, Supplier<Boolean> bPressed, Supplier<Boolean> yPressed,
            Supplier<Boolean> manualOverride,
            Telemetry telemetry) {

        this.beltSubsystem = belt;
        this.intakeSubsystem = intake;
        this.distanceSensor = distanceSensor;
        this.webcam = webcam;
        this.pipeline = pipeline;

        this.rbPressedSupplier = rbPressed;
        this.bPressedSupplier = bPressed;
        this.yPressedSupplier = yPressed;
        this.manualOverrideSupplier = manualOverride;

        this.telemetry = telemetry;

        addRequirements(belt);
    }

    @Override
    public void initialize() {
        webcam.openCameraDeviceAsync(new OpenCvWebcam.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.setPipeline(pipeline);
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) { }
        });

        telemetry.addLine("LaunchSequenceCommand initialized");
        telemetry.update();
    }

    @Override
    public void execute() {

        boolean rbPressed = rbPressedSupplier.get();
        boolean bPressed = bPressedSupplier.get();
        boolean yPressed = yPressedSupplier.get();
        boolean manualOverride = manualOverrideSupplier.get();
        long currentTime = System.currentTimeMillis();

        // --- Manual override skips automatic belt ---
        if (manualOverride) {
            telemetry.addData("Manual Override", "Active - skipping auto belt");
            telemetry.update();
            bPreviouslyPressed = bPressed;
            return;
        }

        // --- Detect B → Hold Y pause sequence ---
        if (!pauseActive) {

            // Step 1: Detect a NEW press of B (rising edge)
            if (bPressed && !bPreviouslyPressed) {
                waitingForY = true;
            }

            // Step 2: If Y is held while waitingForY = true → trigger pause immediately
            if (waitingForY && yPressed) {

                pauseActive = true;
                pauseStartTime = currentTime;
                waitingForY = false;

                beltSubsystem.stop();
                intakeSubsystem.stop();

                if (!viewportPaused) {
                    webcam.pauseViewport();
                    viewportPaused = true;
                }

                telemetry.addData("Pause", "Activated (B then Hold Y)");
                telemetry.update();
            }
        }

        // --- Pause Mode ---
        if (pauseActive) {

            beltSubsystem.stop();
            intakeSubsystem.stop();

            long elapsed = currentTime - pauseStartTime;

            telemetry.addData("Pause Active", true);
            telemetry.addData("Time until resume (ms)", PAUSE_DURATION_MS - elapsed);
            telemetry.update();

            if (elapsed >= PAUSE_DURATION_MS) {

                pauseActive = false;

                amountOfBalls = 0;
                ballPreviouslyDetected = false;
                lastKnownX = 0;
                lastKnownY = 0;
                lastDetectionTime = 0;

                state = State.WAITING_FOR_BALL;
                waitingForFirstFrameAfterPause = true;

                if (viewportPaused) {
                    webcam.resumeViewport();
                    viewportPaused = false;
                }

                telemetry.addData("Pause", "Ended");
                telemetry.addData("Camera", "Resumed");
                telemetry.update();
            }

            bPreviouslyPressed = bPressed;
            return;
        }

        // --- Skip belt logic until first frame after pause ---
        if (waitingForFirstFrameAfterPause) {
            if (!viewportPaused) {
                telemetry.addData("Waiting", "First frame after resume...");
                telemetry.update();
                waitingForFirstFrameAfterPause = false;
            }
        }

        // --- RB must be held for auto sequence ---
        if (!rbPressed) {
            beltSubsystem.stop();
            intakeSubsystem.stop();
            bPreviouslyPressed = bPressed;
//            telemetry.addData("RB Pressed", false);
            telemetry.update();
            return;
        }

        // --- BALL COUNT VIA DISTANCE SENSOR ---
        double distanceCm = distanceSensor.getDistance(DistanceUnit.CM);
        boolean ballDetected = distanceCm < 18.0;

        if (ballDetected && !ballPreviouslyDetected) amountOfBalls++;
        ballPreviouslyDetected = ballDetected;

        boolean intakeRunning = amountOfBalls <= 2;

        if (intakeRunning) intakeSubsystem.intake();
        else {
            intakeSubsystem.stop();
            beltSubsystem.stop();
        }

        telemetry.addData("The amount of balls is ", amountOfBalls);
        telemetry.addData("Intake Running", intakeRunning);

        if (!intakeRunning) {
            bPreviouslyPressed = bPressed;
            telemetry.update();
            return;
        }

        // --- IMAGE PROCESSING: FIND FIRST BALL ---
        List<PurpleBallPipeline.BallData> balls = new ArrayList<>();
        synchronized (pipeline) {
            balls.addAll(pipeline.detectedBalls);
            balls.addAll(pipeline.greenBalls);
        }

        PurpleBallPipeline.BallData firstBall = null;

        for (PurpleBallPipeline.BallData ball : balls) {
            if (ball != null && ball.center != null && "Left".equals(ball.region)) {
                if (firstBall == null || ball.center.x < firstBall.center.x)
                    firstBall = ball;
            }
        }

        if (firstBall != null) {
            lastKnownX = firstBall.center.x;
            lastKnownY = firstBall.center.y;
            lastDetectionTime = currentTime;
        }

        // --- BELT STATE MACHINE ---
        switch (state) {

            case WAITING_FOR_BALL:
                if (firstBall != null) {
                    state = State.ADJUSTING_FIRST_BALL;
                }
                break;

            case ADJUSTING_FIRST_BALL:
                if ((lastKnownX < FIRST_BALL_X_TARGET || lastKnownY < FIRST_BALL_Y_TARGET)
                        && (currentTime - lastDetectionTime <= DETECTION_TIMEOUT)) {

                    beltSubsystem.move_belt(BeltSubsystem.Direction.UTR);

                } else {

                    beltSubsystem.stop();

                    if (lastKnownX >= FIRST_BALL_X_TARGET && lastKnownY >= FIRST_BALL_Y_TARGET) {
                        state = State.FIRST_BALL_READY;
                    }
                }
                break;

            case FIRST_BALL_READY:
                beltSubsystem.stop();
                break;
        }

        // --- TELEMETRY ---
        telemetry.addData("Detected X", lastKnownX);
        telemetry.addData("Detected Y", lastKnownY);
        telemetry.addData("Target X", FIRST_BALL_X_TARGET);
        telemetry.addData("Target Y", FIRST_BALL_Y_TARGET);
        telemetry.addData("Current State", state);

        boolean beltMoving = (state == State.ADJUSTING_FIRST_BALL &&
                (lastKnownX < FIRST_BALL_X_TARGET || lastKnownY < FIRST_BALL_Y_TARGET));

        telemetry.addData("Belt moving?", beltMoving ? "YES" : "NO");
        telemetry.update();

        bPreviouslyPressed = bPressed;
    }

    @Override
    public void end(boolean interrupted) {
//        telemetry.addData("LaunchSequenceCommand ended",
//                interrupted ? "interrupted" : "completed");
        telemetry.update();
        beltSubsystem.stop();
        intakeSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
