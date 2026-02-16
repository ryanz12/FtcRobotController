package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AutoSystems.Intake;
import org.firstinspires.ftc.teamcode.AutoSystems.Ramp;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Arrays;

@Config
@Autonomous(name = "TestAuto", group = "Test")
public class TestAuto extends LinearOpMode {

    @Config
    public static class TestConfig {
        public static double DISTANCE = -30.0;    // How many inches to drive forward
        public static double SPEED = 15.0;       // Slow speed to watch the rollers work
        public static double INTAKE_POWER = 0.8; // Base power for dynamicRoll
        public static double RAMP_POWER = 0.95;  // Keep this slightly higher than intake
        public static double DURATION_RAMP = 4.0;     // How long to keep rollers spinning
        public static double DURATION_INTAKE =4.0;
    }

    private Ramp ramp;
    private Intake intake;
    private MecanumDrive drive;
    private Servo wallServo;

    @Override
    public void runOpMode() {
        ramp = new Ramp(hardwareMap);
        intake = new Intake(hardwareMap);
        wallServo = hardwareMap.get(Servo.class, "wallServo");

        // We start at 0,0,0 so "forward" is just increasing X
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive = new MecanumDrive(hardwareMap, startPose);

        VelConstraint slowVel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(TestConfig.SPEED)
        ));

        // THE TEST: Lower wall, then move forward while spinning everything
        Action testAction = new SequentialAction(
                // 1. Prep
                (telemetryPacket) -> {
                    wallServo.setPosition(0.0); // Lower wall
                    return false; // Run once and finish
                },

                // 2. The Move + Intake
                new ParallelAction(
                        drive.actionBuilder(startPose)
                                .lineToX(TestConfig.DISTANCE, slowVel)
                                .build(),
                        intake.dynamicRoll(TestConfig.DURATION_INTAKE, TestConfig.INTAKE_POWER),
                        ramp.rampUp(TestConfig.DURATION_RAMP, TestConfig.RAMP_POWER)
                ),

                // 3. Lock the balls in
                (telemetryPacket) -> {
                    wallServo.setPosition(0.5); // Raise wall
                    return false;
                }
        );

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(testAction);
    }
}