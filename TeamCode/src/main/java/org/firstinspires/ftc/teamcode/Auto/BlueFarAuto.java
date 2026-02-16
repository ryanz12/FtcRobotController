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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AutoSystems.Intake;
import org.firstinspires.ftc.teamcode.AutoSystems.Launcher;
import org.firstinspires.ftc.teamcode.AutoSystems.Outtake;
import org.firstinspires.ftc.teamcode.AutoSystems.Ramp;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Arrays;

@Autonomous(name = "BlueFarAuto", group = "Autonomous")
public class BlueFarAuto extends LinearOpMode {

    @Config
    public static class BlueAutoConfig {
        public static double TRAVEL_VEL = 70.0;
        public static double COLLECT_VEL = 15.0;
        public static double COLLECT_WAIT = 0.2;
        public static double BACKOFF_DIST = 5.0;

        public static double WALL_SERVO_PICKUP = 0.0;
        public static double WALL_SERVO_SHOOT = 0.5;

        // --- Mirrored Poses (Y and H inverted from Red) ---
        public static double START_X = -63;
        public static double START_Y = -12;
        public static double START_H = 0;

        public static double SCORE_X = -60;
        public static double SCORE_Y = -10;
        public static double SCORE_H = -25; // Inverted from -25
        public static double SCORE_Y_OFFSET = 4.0; // Inverted from -4.0

        public static double P2_X = -33;
        public static double P2_COLLECT_Y = 53;
        public static double P4_X = -7;
        public static double P4_COLLECT_Y = 53;

        public static double LAUNCHER_VEL = -500;
        public static double VEL_THRESHOLD = 25;
        public static double RAMP_POWER = 0.7;
        public static double SCORE_DWELL = 3.0;
    }

    private Launcher launcher;
    private Ramp ramp;
    private Intake intake;
    private Outtake outtake;
    private MecanumDrive drive;
    private Servo wallServo;
    private DcMotorSimple light;

    @Override
    public void runOpMode() {
        launcher = new Launcher(hardwareMap);
        ramp = new Ramp(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        wallServo = hardwareMap.get(Servo.class, "wallServo");
        light = hardwareMap.get(DcMotorSimple.class, "light");

        Pose2d initialPose = new Pose2d(BlueAutoConfig.START_X, BlueAutoConfig.START_Y, Math.toRadians(BlueAutoConfig.START_H));
        drive = new MecanumDrive(hardwareMap, initialPose);

        VelConstraint fastVel = new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(BlueAutoConfig.TRAVEL_VEL)));
        VelConstraint slowVel = new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(BlueAutoConfig.COLLECT_VEL)));

        // Phase 1: Initial Score
        Action phase1 = drive.actionBuilder(initialPose)
                .afterTime(0, () -> wallServo.setPosition(BlueAutoConfig.WALL_SERVO_SHOOT))
                .strafeToLinearHeading(new Vector2d(BlueAutoConfig.SCORE_X, BlueAutoConfig.SCORE_Y), Math.toRadians(BlueAutoConfig.SCORE_H), fastVel)
                .waitSeconds(1.5)
                .stopAndAdd(new ParallelAction(ramp.rampUp(BlueAutoConfig.SCORE_DWELL, 1), outtake.roll(BlueAutoConfig.SCORE_DWELL, 1)))
                .build();

        // Phase 2: Collection 1
        Action phase2 = drive.actionBuilder(new Pose2d(BlueAutoConfig.SCORE_X, BlueAutoConfig.SCORE_Y, Math.toRadians(BlueAutoConfig.SCORE_H)))
                .afterTime(0, () -> wallServo.setPosition(BlueAutoConfig.WALL_SERVO_PICKUP))
                .setTangent(Math.toRadians(-90)) // Inverted tangent
                .strafeToLinearHeading(new Vector2d(BlueAutoConfig.P2_X, 12), Math.toRadians(-90), fastVel)
                .strafeTo(new Vector2d(BlueAutoConfig.P2_X, BlueAutoConfig.P2_COLLECT_Y), slowVel)
                .build();

        // Phase 3: Return Score 1
        Action phase3 = drive.actionBuilder(new Pose2d(BlueAutoConfig.P2_X, BlueAutoConfig.P2_COLLECT_Y - BlueAutoConfig.BACKOFF_DIST, Math.toRadians(-90)))
                .afterTime(0, () -> wallServo.setPosition(BlueAutoConfig.WALL_SERVO_SHOOT))
                .strafeToLinearHeading(new Vector2d(BlueAutoConfig.SCORE_X, BlueAutoConfig.SCORE_Y), Math.toRadians(BlueAutoConfig.SCORE_H), fastVel)
                .stopAndAdd(new ParallelAction(ramp.rampUp(BlueAutoConfig.SCORE_DWELL, 1), outtake.roll(BlueAutoConfig.SCORE_DWELL, 1),intake.roll(BlueAutoConfig.SCORE_DWELL,.7)))
                .build();

        // Phase 4: Collection 2
        Action phase4 = drive.actionBuilder(new Pose2d(BlueAutoConfig.SCORE_X, BlueAutoConfig.SCORE_Y, Math.toRadians(BlueAutoConfig.SCORE_H)))
                .afterTime(0, () -> wallServo.setPosition(BlueAutoConfig.WALL_SERVO_PICKUP))
                .setTangent(Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(BlueAutoConfig.P4_X, 17), Math.toRadians(-90), fastVel)
                .strafeTo(new Vector2d(BlueAutoConfig.P4_X, BlueAutoConfig.P4_COLLECT_Y), slowVel)
                .build();

        // Phase 5: Final Score (No Park)
        Action phase5 = drive.actionBuilder(new Pose2d(BlueAutoConfig.P4_X, BlueAutoConfig.P4_COLLECT_Y - BlueAutoConfig.BACKOFF_DIST, Math.toRadians(-90)))
                .afterTime(0, () -> wallServo.setPosition(BlueAutoConfig.WALL_SERVO_SHOOT))
                .strafeToLinearHeading(new Vector2d(BlueAutoConfig.SCORE_X, BlueAutoConfig.SCORE_Y + BlueAutoConfig.SCORE_Y_OFFSET), Math.toRadians(BlueAutoConfig.SCORE_H), fastVel)
                .stopAndAdd(new ParallelAction(ramp.rampUp(10.0, 1), outtake.roll(10.0, 1),intake.roll(10.0,1)))
                .build();

        // Light Status Action
        Action lightStatus = (telemetryPacket) -> {
            if (Math.abs(launcher.getVelocity() - BlueAutoConfig.LAUNCHER_VEL) <= BlueAutoConfig.VEL_THRESHOLD) {
                light.setPower(1.0);
            } else {
                light.setPower(0.0);
            }
            return true;
        };

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(new ParallelAction(
                launcher.shoot(RedFarAuto.AutoConfig.LAUNCHER_VEL, 30),
                lightStatus,
                new SequentialAction(
                        phase1,
                        new ParallelAction(phase2, intake.dynamicRoll(5, 0.7), ramp.rampUp(6, RedFarAuto.AutoConfig.RAMP_POWER)),
                        phase3,
                        new ParallelAction(phase4, intake.dynamicRoll(5, 0.7), ramp.rampUp(6, RedFarAuto.AutoConfig.RAMP_POWER)),
                        phase5
                )
        ));
    }
}