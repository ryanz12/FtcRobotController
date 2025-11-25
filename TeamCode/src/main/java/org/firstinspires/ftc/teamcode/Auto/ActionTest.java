package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class ActionTest extends LinearOpMode {
    private DcMotor shootMotor;
    private CRServo rampServoOne;
    private CRServo rampServoTwo;

    private CRServo intakeServo;


    @Override
    public void runOpMode() throws InterruptedException {
        shootMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
        shootMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rampServoOne = hardwareMap.get(CRServo.class,"beltServoOne");
        rampServoTwo = hardwareMap.get(CRServo.class,"beltServoTwo");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        Pose2d beginPos = new Pose2d(new Vector2d(-57,-12),0);
        MecanumDrive drive = new MecanumDrive(hardwareMap,beginPos);

        waitForStart();
        Action action = drive.actionBuilder(beginPos)
                .stopAndAdd(new SequentialAction(
                        new Outake(shootMotor,true),
                        new Intake(intakeServo,1),
                        new Ramp(rampServoOne,rampServoTwo, Ramp.Direction.UP)
                ))
                .build();
    }
}
