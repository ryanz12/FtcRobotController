package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
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

        initialPose = new Pose2d(63, 12, Math.toRadians(180));
        drive = new MecanumDrive(hardwareMap, initialPose);

        VelConstraint baseVel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(50.0),          // in/s
                new AngularVelConstraint(Math.toRadians(180))  // rad/s
        ));
        AccelConstraint baseAccel = new ProfileAccelConstraint(-30.0, 30.0); // in/s^2

        Action test = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(39, 12), Math.toRadians(180),
                        // override ONLY velocity for this segment (slow)
                        new TranslationalVelConstraint(20.0),
                        // keep base accel
                        baseAccel
                )
                .build();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(
                test
            )
        );

    }
}
