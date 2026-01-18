package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
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

@Config
@Autonomous
public class RedCloseAuto extends LinearOpMode {

    private Launcher launcher;
    private Ramp ramp;
    private Intake intake;
    private Outtake outtake;
    private MecanumDrive drive;
    private Pose2d initialPose;

    @Override
    public void runOpMode(){
        launcher = new Launcher(hardwareMap);
        ramp = new Ramp(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);

        initialPose = new Pose2d(50, -50, Math.toRadians(-45));
        drive = new MecanumDrive(hardwareMap, initialPose);

        VelConstraint baseVel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(50.0),          // in/s
                new AngularVelConstraint(Math.toRadians(180))  // rad/s
        ));
        AccelConstraint baseAccel = new ProfileAccelConstraint(-30.0, 30.0); // in/s^2

        Action phase1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(20, -52))
                .turn(Math.toRadians(45))
                .waitSeconds(0.5)
                .stopAndAdd(
                        new ParallelAction(
                                ramp.rampUp(4, 1),
                                outtake.roll(4, 1)
                        )
                )
                .build();

        Action phase2 = drive.actionBuilder(new Pose2d(12, -52, 0))
                .turn(Math.toRadians(135))
                .strafeTo(new Vector2d(5, -60))
                .strafeTo(new Vector2d(30, -65))
                .build();

        Action phase3 = drive.actionBuilder(new Pose2d(20, -65, Math.toRadians(135)))
                .strafeTo(new Vector2d(12, -52))
                .waitSeconds(0.5)
                .stopAndAdd(
                        new ParallelAction(
                                ramp.rampUp(4, 1),
                                outtake.roll(4, 1)
                        )
                )
                .build();

        Action phase4 = drive.actionBuilder(new Pose2d(12, -52, Math.toRadians(135)))
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
                                    ramp.rampUp(5, 0.6),
                                    outtake.roll(5, -0.55)
                            ),
                            phase3
                    ),
                    launcher.shoot(-1150, 30)
                )
            )
        );

    }
}
