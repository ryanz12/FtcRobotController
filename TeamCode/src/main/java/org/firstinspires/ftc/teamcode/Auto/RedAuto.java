package org.firstinspires.ftc.teamcode.Auto;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
@Autonomous(name = "AutoRed",group = "Left")
//
public class RedAuto extends LinearOpMode {
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
        //This is purely to test whether auto is working
        Action path = drive.actionBuilder(beginPos)
                .lineToX(53)
                .waitSeconds(2)
                .turn(Math.toRadians(90))
                .strafeTo(new Vector2d(-12,-12))
                .turn(Math.toRadians(180))
                .lineToY(-40)
                .build();
        Actions.runBlocking(new SequentialAction(path));

    }
}
