package org.firstinspires.ftc.teamcode.Auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Vector2d;
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
public class RedAutoClose extends LinearOpMode {

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
            target_velocity = target;

            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    if (!initialized){
                        timer.reset();
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

        initialPose = new Pose2d(-49.5, 49.4, Math.toRadians(135));
        drive = new MecanumDrive(hardwareMap, initialPose);

        // Move to position to shoot
        Action phase1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-11.5, 14.6))
                .build();

        // Shoot the balls
        Action phase2 = drive.actionBuilder(new Pose2d(-5.5, 14.6, Math.toRadians(135)))
                .waitSeconds(2)
                .stopAndAdd(
                        new ParallelAction(
                                ramp.rampUp(3, 1),
                                intake.roll(3)
                        )
                )
                .build();

        // Turn to face balls
        Action phase3 = drive.actionBuilder(new Pose2d(-5.5, 14.6, Math.toRadians(135)))
                .turn(Math.toRadians(135))
                .strafeTo(new Vector2d(2, 62.5))
                .build();

        // Return to shoot
        Action phase4 = drive.actionBuilder(new Pose2d(-11.5, 14.6, Math.toRadians(270)))
                .turn(Math.toRadians(-135))
                .stopAndAdd(
                        new ParallelAction(
                                ramp.rampUp(3, 1),
                                intake.roll(3)
                        )
                )
                .build();


        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(
                phase1,
                new ParallelAction(
                        launcher.shoot(-1200, 5),
                        phase2
                ),
                new ParallelAction(
                        phase3,
                        intake.roll(5),
                        ramp.rampUp(5, 0.5)
                ),
                new ParallelAction(
                        phase4,
                        launcher.shoot(-1250, 5)
                )
            )
        );

    }
}

