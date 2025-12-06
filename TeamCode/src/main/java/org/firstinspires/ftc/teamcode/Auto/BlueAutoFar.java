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
public class BlueAutoFar extends LinearOpMode {

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

        initialPose = new Pose2d(61.5, -12.5, Math.toRadians(-180));
        drive = new MecanumDrive(hardwareMap, initialPose);

        Action phase1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(58, -12.5))
                .turn(Math.toRadians(25))
                .waitSeconds(0.5)
                .stopAndAdd(
                        new ParallelAction(
                                ramp.rampUp(6, 1),
                                intake.roll(6, 0.3)
                        )
                )
                .build();

        Action phase2 = drive.actionBuilder(new Pose2d(58, -12.5, Math.toRadians(-155)))
                .turn(Math.toRadians(-115)) //turn to 270 deg
                .strafeTo(new Vector2d(31, -29))
                .strafeTo(new Vector2d(31, -68.5))
                .build();

        Action phase3 = drive.actionBuilder(new Pose2d(58, -12.5, Math.toRadians(-270)))
                .strafeTo(new Vector2d(52, -22))
                .build();

        Action phase4 = drive.actionBuilder(new Pose2d(52, -22, Math.toRadians(-270)))
                .turn(Math.toRadians(120))
                .waitSeconds(1)
                .stopAndAdd(
                        new ParallelAction(
                                ramp.rampUp(7, 1),
                                intake.roll(7, 0.3)
                        )
                )
                .build();

        Action phase5 = drive.actionBuilder(new Pose2d(52, -22, Math.toRadians(-150)))
                .strafeTo(new Vector2d(35, -67.5))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(
                        new ParallelAction(
                                phase1,
                                launcher.shoot(-1600, 7)
                        ),
                        new ParallelAction(
                                phase2,
                                intake.roll(5, 0.8),
                                ramp.rampUp(5, 0.5),
                                launcher.shoot(200, 5)
                        ),
                        phase3,
                        new ParallelAction(
                                phase4,
                                launcher.shoot(-1500, 7)
                        ),
                        new ParallelAction(
                                phase5
//                              launcher.shoot(0, 3)
                        )
                )
        );
    }
}

