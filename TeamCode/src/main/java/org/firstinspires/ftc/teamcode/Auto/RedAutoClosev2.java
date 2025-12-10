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
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous
public class RedAutoClosev2 extends LinearOpMode {

    private class Launcher {
        private DcMotorEx launch_motor;
        private ElapsedTime timer;
        private VoltageSensor voltage_sensor;
        private double current_velocity;
        private double target_velocity;

        private double kP = 0.003;
        private double kI = 0;
        private double kD = 0.01;
        private double kF = 0.0004167;
        private double voltage_constant = 0.95;
        private double voltage_reference = 13.0;
        private double voltage;
        private double error, error_sum, prev_error, delta_error, delta_time;
        private double output;

        public Launcher (HardwareMap hwMap){
            launch_motor = hwMap.get(DcMotorEx.class, "launchMotor");
            launch_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            launch_motor.setDirection(DcMotorSimple.Direction.REVERSE);
            launch_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            launch_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            voltage_sensor = hwMap.get(VoltageSensor.class, "Control Hub");
            timer = new ElapsedTime();
        }

        private double calc_power(double t){
            current_velocity = launch_motor.getVelocity();

            error = target_velocity-current_velocity;
            error_sum += error;

            delta_time = t > 0 ? t : 1e-3;
            delta_error = (error-prev_error)/delta_time;
            prev_error = error;

            voltage = voltage_sensor.getVoltage();

            double ffv = kF * target_velocity * (voltage_reference/voltage)*voltage_constant;
            double ff = kF * target_velocity;

            output = ff + kP * error + kI * error_sum + kD * delta_error;

            return -output;
        }

        public Action shoot(double target, double time){
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    if (!initialized){
                        timer.reset();
                        target_velocity = target;
                        initialized = true;
                    }

                    double current_time = timer.seconds();

                    if (current_time >= time){
                        launch_motor.setPower(0);
                        error = error_sum = prev_error = delta_error = delta_time = 0;

                        return false;
                    }

                    double power = calc_power(current_time);
                    launch_motor.setPower(power);

                    telemetryPacket.put("current velocity", launch_motor.getVelocity());
                    telemetryPacket.put("target velocity", target_velocity);
                    telemetryPacket.put("power", power);
                    telemetryPacket.put("time", timer.seconds());

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
        private DcMotorEx intakeServo;
        private ElapsedTime timer;

        public Intake(HardwareMap hwMap){
            intakeServo = hwMap.get(DcMotorEx.class, "intakeServo");
            timer = new ElapsedTime();
        }

        public Action roll(double seconds, double power){
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    if (!initialized){
                        intakeServo.setPower(-power);
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

        initialPose = new Pose2d(-50, 50, Math.toRadians(135));
        drive = new MecanumDrive(hardwareMap, initialPose);

        // Drive to shooting pose
        Action phase1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(2, 12.5))
                .build();

        Action phase2 = drive.actionBuilder(new Pose2d(2, 12.5, Math.toRadians(135)))
                .waitSeconds(2)
                .stopAndAdd(
                        new ParallelAction(
                                ramp.rampUp(4, 1),
                                intake.roll(4, 0.2)
                        )
                )
                .build();


        // Drive to pick up balls
        Action phase3 = drive.actionBuilder(new Pose2d(2, 12.5, Math.toRadians(135)))
                .turn(Math.toRadians(135))
                .strafeTo(new Vector2d(-6, 65.5))
                .build();

        Action phase4 = drive.actionBuilder(new Pose2d(-2, 62.5, Math.toRadians(270)))
                .strafeTo(new Vector2d(-6, 16.5))
                .build();


        // Rotate then shoot
        Action phase5 = drive.actionBuilder(new Pose2d(-6, 16.5, Math.toRadians(270)))
                .turn(Math.toRadians(-135))
                .build();

        Action phase6 = drive.actionBuilder(new Pose2d(-6, 16.5, Math.toRadians(135)))
                .waitSeconds(2)
                .stopAndAdd(
                        new ParallelAction(
                                ramp.rampUp(5, 1),
                                intake.roll(5, 0.3)
                        )
                )
                .build();

        Action phase7 = drive.actionBuilder(new Pose2d(2, 12.5, Math.toRadians(135)))
                .turn(Math.toRadians(135))
                .strafeTo(new Vector2d(35, 28))
//              .strafeTo(new Vector2d(35, 67.5))
                .build();

        Action phase8 = drive.actionBuilder(new Pose2d(25, 62.5, Math.toRadians(270)))
                .strafeTo(new Vector2d(2, 12.5))
                .turn(Math.toRadians(-135))
                .build();

        Action phase9 = drive.actionBuilder(new Pose2d(2, 12.5, Math.toRadians(-135)))
                .waitSeconds(3.5)
                .stopAndAdd(
                        new ParallelAction(
                                ramp.rampUp(2, 1),
                                intake.roll(2, 0.3)
                        )
                )
                .build();

        Action phase10 = drive.actionBuilder(new Pose2d(35, 67.5, Math.toRadians(135)))
                .strafeTo(new Vector2d(60, 60))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        phase1,
                        phase2,
                        launcher.shoot(-1250, 6)
                ),
                new ParallelAction(
                        phase3,
                        ramp.rampUp(5, 0.5),
                        intake.roll(5, 0.5),
                        launcher.shoot(200, 5)
                ),
                phase4,
                new ParallelAction(
                        phase5,
                        phase6,
                        launcher.shoot(-1250, 5)
                ),
                new ParallelAction(
                        phase7
//                      ramp.rampUp(5, 0.5),
//                      intake.roll(5, 0.5),
//                      launcher.shoot(0, 5)
                )
//              new ParallelAction(
//                      phase8,
//                      launcher.shoot(-1300, 4),
//                      phase9
//              ),
//              phase10
            )
        );

    }
}
