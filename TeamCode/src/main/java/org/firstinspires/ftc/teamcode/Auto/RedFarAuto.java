package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoSystems.Intake;
import org.firstinspires.ftc.teamcode.AutoSystems.Launcher;
import org.firstinspires.ftc.teamcode.AutoSystems.Outtake;
import org.firstinspires.ftc.teamcode.AutoSystems.Ramp;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Arrays;

@Autonomous
public class RedFarAuto extends LinearOpMode {
    private Launcher launcher;
    private Ramp ramp;
    private Intake intake;
    private Outtake outtake;
    private MecanumDrive drive;
    private Pose2d initialPose;

    @Override
    public void runOpMode(){
        // initialize
        launcher = new Launcher(hardwareMap);
        ramp = new Ramp(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);

        initialPose = new Pose2d(-63, 12, 0);
        drive = new MecanumDrive(hardwareMap, initialPose);

        VelConstraint slowVel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(25.0),                // in/s
                new AngularVelConstraint(Math.toRadians(120))       // rad/s
        ));

        AccelConstraint slowAccel = new ProfileAccelConstraint(-20.0, 20.0);


        Action phase1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-60, 10))
                .turn(Math.toRadians(-25))
                .waitSeconds(0.75)
                .stopAndAdd(
                        new ParallelAction(
                                ramp.rampUp(4, 1),
                                outtake.roll(4, 1)
                        )
                )
                .build();

        Action phase2 = drive.actionBuilder(new Pose2d(-60, 10, Math.toRadians(-25)))
                .turn(Math.toRadians(115))
                .strafeTo(new Vector2d(-45, 0))
                .strafeTo(new Vector2d(-45, -22))
                .build();

        Action phase3 = drive.actionBuilder(new Pose2d(-45, -24, Math.toRadians(90)))
                .strafeTo(new Vector2d(-60, 10))
                .turn(Math.toRadians(-115))
                .stopAndAdd(
                        new ParallelAction(
                                ramp.rampUp(3, 1),
                                outtake.roll(3, 1),
                                intake.roll(3, 1)
                        )
                )
                .build();

        Action phase4 = drive.actionBuilder(new Pose2d(-60, 10, Math.toRadians(-25)))
                .turn(Math.toRadians(115))
                .strafeTo(new Vector2d(-26, 0))
                .strafeTo(new Vector2d(-26, -24))
                .build();

        Action phase5 = drive.actionBuilder(new Pose2d(-25, -28, Math.toRadians(90)))
                .strafeTo(new Vector2d(-60, 4))
                .turn(Math.toRadians(-115))
                .stopAndAdd(
                        new ParallelAction(
                                ramp.rampUp(3, 1),
                                outtake.roll(3, 1),
                                intake.roll(3, 1)
                        )
                )
                .build();

        Action park = drive.actionBuilder(new Pose2d(-60, 10, Math.toRadians(-25)))
                .strafeTo(new Vector2d(25, -26))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        new SequentialAction(
                                phase1,
                                new ParallelAction(
                                        phase2,
                                        intake.roll(5, 0.9),
                                        ramp.rampUp(5, 0.3),
                                        outtake.roll(5, -1)
                                ),
                                phase3,
                                new ParallelAction(
                                        phase4,
                                        intake.roll(5.5, 0.9),
                                        ramp.rampUp(5, 0.3),
                                        outtake.roll(5, -1)
                                ),
                                phase5,
                                park
                        ),
                        launcher.shoot(-1520, 30)
                )
            )
        );

    }
}
