package org.firstinspires.ftc.teamcode.pedroPathing.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "blue test", group = "Autonomous")
public class Bluetest extends LinearOpMode {
    private MecanumDrive drive;
    private Pose2d beginPose;
    private DcMotor rotateMotor, slideMotor;
    private CRServo intakeServo;
    private DistanceSensor distanceSensor;

    // actionBuilder builds from the drive steps passed to it
    @Override
    public void runOpMode() {
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        rotateMotor = hardwareMap.get(DcMotorEx.class, "rotateMotor");
        rotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        beginPose = new Pose2d(0, 0, 0);
        drive = new MecanumDrive(hardwareMap, beginPose);

        Pose2d initialPose = new Pose2d(-35, -62, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        //1  Block
        // goes to the bucket
        Action phase1 = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(-54, -54))
                .turn(Math.toRadians(45))
                .build();

        // scores
        Action phase2 = drive.actionBuilder(new Pose2d(-54, -53, Math.toRadians(-135)))
                .stopAndAdd(new Slide(slideMotor, 1800))
                .strafeTo(new Vector2d(-56, -57))
                .waitSeconds(0.25)
                .stopAndAdd(new Intake(intakeServo, 1))
                .waitSeconds(0.35)
                .stopAndAdd(new Intake(intakeServo, 0))
                .stopAndAdd(new Slide(slideMotor, 0))
                .build();

        Action lower_arm = drive.actionBuilder(new Pose2d(-58, -56, Math.toRadians(-135)))
                .waitSeconds(1)
                .stopAndAdd(new Arm(rotateMotor, 200))
                .build();

        // strafes to right sample position
        Action phase3 = drive.actionBuilder(new Pose2d(-58, -56, Math.toRadians(-135)))
                .turn(Math.toRadians(-135))
                .strafeTo(new Vector2d(-48, -40))
                .build();

        // retrieves sample and goes back to score
        Action phase4 = drive.actionBuilder(new Pose2d(-48, -40, Math.toRadians(90)))
                .stopAndAdd(new Slide(slideMotor, 1000))
                .waitSeconds(1.25)
                .strafeTo(new Vector2d(-54, -54))
                .turn(Math.toRadians(135))
                .stopAndAdd(new Slide(slideMotor, 0))
                .waitSeconds(0.5)
                .stopAndAdd(new Arm(rotateMotor, 1250))
                .waitSeconds(1)
                .stopAndAdd(new Slide(slideMotor, 1850))
                .strafeTo(new Vector2d(-52, -58))
                .waitSeconds(0.5)
                .stopAndAdd(new Intake(intakeServo, 1))
                .waitSeconds(0.25)
                .stopAndAdd(new Intake(intakeServo, 0))
                .stopAndAdd(new Slide(slideMotor, 0))
                .build();

        // turn and go to middle sample
        Action phase5 = drive.actionBuilder(new Pose2d(-58, -56, Math.toRadians(-135)))
                .turn(Math.toRadians(-135))
                .strafeTo(new Vector2d(-64, -40))
                .build();

        Action lower_arm2 = drive.actionBuilder(new Pose2d(-58, -56, Math.toRadians(-135)))
                .waitSeconds(1)
                .stopAndAdd(new Arm(rotateMotor, 200))
                .build();

        // retrives sample and turns to score
        Action phase6 = drive.actionBuilder(new Pose2d(-58, -40, Math.toRadians(90)))
                .stopAndAdd(new Slide(slideMotor, 1000))
                .waitSeconds(1.25)
                .strafeTo(new Vector2d(-54, -54))
                .turn(Math.toRadians(137))
                .stopAndAdd(new Slide(slideMotor, 0))
                .waitSeconds(0.5)
                .stopAndAdd(new Arm(rotateMotor, 1250))
                .waitSeconds(1)
                .stopAndAdd(new Slide(slideMotor, 1850))
                .strafeTo(new Vector2d(-52, -58))
                .waitSeconds(0.5)
                .stopAndAdd(new Intake(intakeServo, 1))
                .waitSeconds(0.25)
                .stopAndAdd(new Intake(intakeServo, 0))
                .build();

        Action lower_arm3 = drive.actionBuilder(new Pose2d(-58, -56, Math.toRadians(-137)))
                .stopAndAdd(new Slide(slideMotor, 0))
                .waitSeconds(1)
                .stopAndAdd(new Arm(rotateMotor, 0))
                .build();

        // parks?
        Action phase7 = drive.actionBuilder(new Pose2d(-58, -56, Math.toRadians(-135)))
                .strafeTo(new Vector2d(-37, -10))
                .build();

        while (opModeInInit()){
            telemetry.addData("Distnace (cm) from wall: ", distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }

        waitForStart();

        if(isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(
                        new ParallelAction(
                                phase1,
                                new Arm(rotateMotor, 1250),
                                new Intake(intakeServo, 0)
                        ),
                        phase2,
                        new ParallelAction(
                                phase3,
                                lower_arm
                        ),
                        phase4,
                        new ParallelAction(
                                phase5,
                                lower_arm2
                        ),
                        phase6,
                        new ParallelAction(
                                phase7,
                                lower_arm3
                        )
                )
        );

    }
    // -------------------------Rotate Arm-------------------------
    private class Arm implements Action {
        DcMotor rotateMotor;
        int position;

        public Arm(DcMotor motor, int position){
            this.rotateMotor = motor;
            this.position = position;
        }

        public void goto_position(){
            rotateMotor.setTargetPosition(position);
            rotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rotateMotor.setPower(0.8);
        }

        public boolean reached_position(){
            return Math.abs(position - rotateMotor.getCurrentPosition()) <= 50;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            goto_position();
            return reached_position();
        }
    }

    // -------------------------Linear Slides-------------------------
    private class Slide implements Action {
        DcMotor slideMotor;
        int position;

        public Slide(DcMotor motor, int position){
            this.slideMotor = motor;
            this.position = position;
        }

        public void goto_position(){
            slideMotor.setTargetPosition(position);
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor.setPower(1);
        }

        public boolean reached_position(){
            return Math.abs(position - slideMotor.getCurrentPosition()) <= 50;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            goto_position();
            return reached_position();
        }
    }

    // -------------------------INTAKE-------------------------
    private class Intake implements Action {
        CRServo intakeServo;
        int direction;
        ElapsedTime timer;

        public Intake(CRServo servo, int dir){
            this.intakeServo = servo;
            this.direction = dir;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null){
                timer = new ElapsedTime();
            }
            intakeServo.setPower(direction == 1 ? 1 : -0.7);

            return timer.milliseconds() < 500 ? true : false;
        }
    }

    private class Actuator implements Action {
        DcMotor actuatorMotor;
        ElapsedTime timer;

        public Actuator(DcMotor motor){
            this.actuatorMotor = motor;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null){
                timer = new ElapsedTime();
            }
            actuatorMotor.setPower(1);
            return timer.seconds() < 6 ? true : false;
        }
    }
}