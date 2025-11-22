package org.firstinspires.ftc.teamcode.Auto;
import com.acmerobotics.roadrunner.Action;
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
public class RedAuto extends LinearOpMode {
    private DcMotor shootMotor;
    private CRServo rampServoOne;
    private CRServo rampServoTwo;

    @Override
    public void runOpMode() throws InterruptedException {
        shootMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
        shootMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rampServoOne = hardwareMap.get(CRServo.class,"beltServoOne");
        rampServoTwo = hardwareMap.get(CRServo.class,"beltServoTwo");

        Pose2d beginPos = new Pose2d(new Vector2d(-57,-45),0);
        MecanumDrive drive = new MecanumDrive(hardwareMap,beginPos);

        waitForStart();
        //This is purely to test whether auto is working
        Action path = drive.actionBuilder(beginPos)
                .lineToY(-15)
                .turn(20)
                .stopAndAdd(new SequentialAction(
                        new Outake(shootMotor,true),
                        new Ramp(rampServoOne,rampServoTwo, Ramp.Direction.UP)
                ))
                .strafeTo(new Vector2d(-11,-15))
                .build();
        Actions.runBlocking(new SequentialAction(path));

    }
}
