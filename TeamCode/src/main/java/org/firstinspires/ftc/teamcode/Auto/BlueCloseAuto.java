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

@Autonomous(name = "BlueCloseAuto", group = "Autonomous")
public class BlueCloseAuto extends LinearOpMode {
    @Config
    public static class BlueAutoConfigClose {
        // --- Speed & Timing ---
        public static double TRAVEL_VEL = 60.0;
        public static double SCORE_DWELL = 5.0; // 5s launch sequences
        public static double COLLECT_TIME = 5.0;

        // --- Start Pose ---
        public static double START_X = 50.0;
        public static double START_Y = 50.0;
        public static double START_H = 45.0;

        // --- Phase 1: Score 1 ---
        public static double P1_SCORE_X = 20.0;
        public static double P1_SCORE_Y = 20;
        public static double P1_TURN = -45.0;

        // --- Phase 2: Collect ---
        public static double P2_TURN = -135.0;

        public static double P2_1_COLLECT_X = -8;
        public static double P2_1_COLLECT_Y = 15;

        public static double P2_2_COLLECT_X = -8;

        public static double P2_2_COLLECT_Y = 50;

        // --- Phase 3: Collect ---
        public static double P3_1_COLLECT_X = -35;
        public static double P3_1_COLLECT_Y = 15;
        public static double P3_2_COLLECT_X = -35;

        public static double P3_2_COLLECT_Y = 50;


        // --- Subsystems ---
        public static double LAUNCHER_VEL = -1150.0;
        public static double VEL_THRESHOLD = 25.0;
        public static double RAMP_POWER = 0.7;
        public static double WALL_SERVO_PICKUP = 0.0;
        public static double WALL_SERVO_SHOOT = 0.5;
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

        Pose2d initialPose = new Pose2d(BlueCloseAuto.BlueAutoConfigClose.START_X, BlueCloseAuto.BlueAutoConfigClose.START_Y, Math.toRadians(BlueCloseAuto.BlueAutoConfigClose.START_H));
        drive = new MecanumDrive(hardwareMap, initialPose);

        VelConstraint fastVel = new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(BlueCloseAuto.BlueAutoConfigClose.TRAVEL_VEL)));
        VelConstraint slowVel = new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(RedFarAuto.AutoConfig.COLLECT_VEL)));


        // --- PHASE 1: Initial Score ---
        Action phase1 = drive.actionBuilder(initialPose)
                .afterTime(0, () -> wallServo.setPosition(BlueCloseAuto.BlueAutoConfigClose.WALL_SERVO_SHOOT))
                .strafeTo(new Vector2d(BlueCloseAuto.BlueAutoConfigClose.P1_SCORE_X, BlueCloseAuto.BlueAutoConfigClose.P1_SCORE_Y))
                .stopAndAdd(new ParallelAction(ramp.rampUp(BlueCloseAuto.BlueAutoConfigClose.SCORE_DWELL, 1), outtake.roll(BlueCloseAuto.BlueAutoConfigClose.SCORE_DWELL, 1),intake.roll(BlueCloseAuto.BlueAutoConfigClose.SCORE_DWELL,1)))
                .build();

        Action phase2 = drive.actionBuilder(new Pose2d(BlueCloseAuto.BlueAutoConfigClose.P1_SCORE_X, BlueCloseAuto.BlueAutoConfigClose.P1_SCORE_Y, Math.toRadians(BlueCloseAuto.BlueAutoConfigClose.P1_TURN)))
                .afterTime(0, () -> wallServo.setPosition(BlueCloseAuto.BlueAutoConfigClose.WALL_SERVO_PICKUP))
                .setTangent(Math.toRadians(90))
                // 1. AVOIDANCE MOVE: Strafe to a "Clearance" point first.
                .strafeToLinearHeading(
                        new Vector2d(BlueCloseAuto.BlueAutoConfigClose.P2_1_COLLECT_X, BlueCloseAuto.BlueAutoConfigClose.P2_1_COLLECT_Y),
                        Math.toRadians(-90),
                        fastVel
                )

                // 3. COLLECTION: Final push
                .strafeTo(new Vector2d(BlueCloseAuto.BlueAutoConfigClose.P2_2_COLLECT_X, BlueCloseAuto.BlueAutoConfigClose.P2_2_COLLECT_Y),slowVel)
                .build();

        // --- PHASE 3: Return Score ---
        Action phase3 = drive.actionBuilder(new Pose2d(BlueCloseAuto.BlueAutoConfigClose.P2_2_COLLECT_X, BlueCloseAuto.BlueAutoConfigClose.P2_2_COLLECT_Y, Math.toRadians(BlueCloseAuto.BlueAutoConfigClose.P2_TURN)))
                .afterTime(0, () -> wallServo.setPosition(BlueCloseAuto.BlueAutoConfigClose.WALL_SERVO_SHOOT))
                .strafeToLinearHeading(new Vector2d(BlueCloseAuto.BlueAutoConfigClose.P1_SCORE_X, BlueCloseAuto.BlueAutoConfigClose.P1_SCORE_Y),Math.toRadians(BlueCloseAuto.BlueAutoConfigClose.START_H),fastVel)
                .stopAndAdd(new ParallelAction(ramp.rampUp(BlueCloseAuto.BlueAutoConfigClose.SCORE_DWELL, 1), outtake.roll(BlueCloseAuto.BlueAutoConfigClose.SCORE_DWELL, 1),intake.roll(BlueCloseAuto.BlueAutoConfigClose.SCORE_DWELL,1)))
                .build();

        Action phase4 = drive.actionBuilder(new Pose2d(BlueCloseAuto.BlueAutoConfigClose.P1_SCORE_X, BlueCloseAuto.BlueAutoConfigClose.P1_SCORE_Y,Math.toRadians(BlueCloseAuto.BlueAutoConfigClose.START_H)))
                .afterTime(0, () -> wallServo.setPosition(BlueCloseAuto.BlueAutoConfigClose.WALL_SERVO_PICKUP))
                .setTangent(Math.toRadians(90))
                // 1. AVOIDANCE MOVE: Strafe to a "Clearance" point first.
                .strafeToLinearHeading(
                        new Vector2d(BlueCloseAuto.BlueAutoConfigClose.P3_1_COLLECT_X, BlueCloseAuto.BlueAutoConfigClose.P3_1_COLLECT_Y),
                        Math.toRadians(-90),
                        fastVel
                )

                // 3. COLLECTION: Final push
                .strafeTo(new Vector2d(BlueCloseAuto.BlueAutoConfigClose.P3_2_COLLECT_X, BlueCloseAuto.BlueAutoConfigClose.P3_2_COLLECT_Y),slowVel)
                .build();

        Action phase5 = drive.actionBuilder(new Pose2d(BlueCloseAuto.BlueAutoConfigClose.P3_2_COLLECT_X, BlueCloseAuto.BlueAutoConfigClose.P3_2_COLLECT_Y, Math.toRadians(BlueCloseAuto.BlueAutoConfigClose.START_H)))
                .afterTime(0, () -> wallServo.setPosition(BlueCloseAuto.BlueAutoConfigClose.WALL_SERVO_SHOOT))
                .strafeToLinearHeading(new Vector2d(BlueCloseAuto.BlueAutoConfigClose.P1_SCORE_X, BlueCloseAuto.BlueAutoConfigClose.P1_SCORE_Y+12 ),Math.toRadians(BlueCloseAuto.BlueAutoConfigClose.START_H),fastVel)
                .stopAndAdd(new ParallelAction(ramp.rampUp(BlueCloseAuto.BlueAutoConfigClose.SCORE_DWELL, 1), outtake.roll(BlueCloseAuto.BlueAutoConfigClose.SCORE_DWELL, 1),intake.roll(BlueCloseAuto.BlueAutoConfigClose.SCORE_DWELL,1)))
                .build();
        // Light Status Action
        Action lightStatus = (p) -> {
            if (Math.abs(launcher.getVelocity() - BlueCloseAuto.BlueAutoConfigClose.LAUNCHER_VEL) <= BlueCloseAuto.BlueAutoConfigClose.VEL_THRESHOLD) light.setPower(1);
            else light.setPower(0);
            return true;
        };

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(new ParallelAction(
                launcher.shoot(BlueCloseAuto.BlueAutoConfigClose.LAUNCHER_VEL, 30),
                lightStatus,
                new SequentialAction(
                        phase1,
                        new ParallelAction(phase2, intake.dynamicRoll(BlueCloseAuto.BlueAutoConfigClose.COLLECT_TIME, 1.0), ramp.rampUp(BlueCloseAuto.BlueAutoConfigClose.COLLECT_TIME, 1.0)),
                        phase3,
                        new ParallelAction(phase4, intake.dynamicRoll(BlueCloseAuto.BlueAutoConfigClose.COLLECT_TIME, 1.0), ramp.rampUp(BlueCloseAuto.BlueAutoConfigClose.COLLECT_TIME, 1.0)),
                        phase5
                )

        ));
    }
}

