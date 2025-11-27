package org.firstinspires.ftc.teamcode.Auto;

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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous
public class BlueAutoFar extends LinearOpMode {

    private class Launcher {
        private DcMotorEx launchMotor;
        private ElapsedTime timer;

        // Initialize
        public Launcher(HardwareMap hwMap){
            launchMotor = hwMap.get(DcMotorEx.class, "launchMotor");
            launchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            timer = new ElapsedTime();
        }


//         Get some action going
        public Action shoot(double target) {
            return new Action(){
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    if (!initialized){
                        launchMotor.setPower(1);
                        initialized = true;
                    }

                    // * Gets the velocity in ticks/second
                    double vel = launchMotor.getVelocity();
                    telemetryPacket.put("velocity", vel);

                    if (vel < target){
                        launchMotor.setPower(0);
                        return false;
                    }

                    return true;
                }
            };
        }

        public Action breakLauncher() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    return false;
                }
            };
        }


        public Action spin(){
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    if (!initialized){
                        timer.reset();
                        launchMotor.setPower(0.72);
                        initialized = true;
                    }

                    double vel = launchMotor.getVelocity();
                    telemetryPacket.put("velocity", vel);

                    if (timer.seconds() > 4.0){
                        launchMotor.setPower(0);
                        return false;
                    }

                    return true;
                }
            };
        }

    }

    private class Ramp {
        private CRServo rampServoOne, rampServoTwo;
        private ElapsedTime timer;

        public Ramp(HardwareMap hwMap){
            rampServoOne = hwMap.get(CRServo.class, "beltServoOne");
            rampServoTwo = hwMap.get(CRServo.class, "beltServoTwo");

            timer = new ElapsedTime();
        }

        public Action rampUp(double seconds, double speed){
            return new Action(){
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    if(!initialized){
                        timer.reset();
                        rampServoOne.setPower(speed);
                        rampServoTwo.setPower(-speed);

                        initialized = true;
                    }

                    telemetryPacket.put("running true", true);

                    if (timer.seconds() > seconds){
                        rampServoOne.setPower(0);
                        rampServoTwo.setPower(0);
                        return false;
                    }

                    return true;
                }
            };
        }
    }

    private class Intake {
        private CRServo intakeServo;
        private ElapsedTime timer;

        public Intake(HardwareMap hwMap){
            intakeServo = hwMap.get(CRServo.class, "intakeServo");
            timer = new ElapsedTime();
        }

        public Action roll(double seconds){
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    if (!initialized){
                        intakeServo.setPower(-1);
                        initialized = true;
                        timer.reset();
                    }

                    if (timer.seconds() > seconds){
                        intakeServo.setPower(0);
                        return false;
                    }

                    return true;
                }
            };
        }
    }

    private MecanumDrive drive;
    private Pose2d initialPose;

    @Override
    public void runOpMode(){
        Launcher launcher = new Launcher(hardwareMap);
        Ramp ramp = new Ramp(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        initialPose = new Pose2d(60, 12, Math.toRadians(160));
        drive = new MecanumDrive(hardwareMap, initialPose);

        Action phase1 = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(36.1, 22), Math.PI/2)
                .turn(Math.PI)
                .strafeTo(new Vector2d(36.1, 60.0))
                .build();

        Action phase2 = drive.actionBuilder(new Pose2d(36.1, 60.8, Math.toRadians(270)))
                .strafeTo(new Vector2d(59.8, 10.4))
                .turn(Math.toRadians(-120))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(
                    launcher.shoot(-1475.0),
                    new ParallelAction(
                            launcher.spin(),
                            ramp.rampUp(4, 1),
                            intake.roll(4)
                    ),
                    new ParallelAction(
                            phase1,
                            intake.roll(8),
                            ramp.rampUp(8, 0.5)
                    ),
                    phase2,
                    launcher.shoot(-1400),
                    new ParallelAction(
                            launcher.spin(),
                            ramp.rampUp(4, 1),
                            intake.roll(4)
                    )
        ));
    }
}
