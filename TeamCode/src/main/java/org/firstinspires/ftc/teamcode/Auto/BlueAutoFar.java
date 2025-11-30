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
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous
public class BlueAutoFar extends LinearOpMode {
    private class Launcher {
        private DcMotorEx launchMotor;
        private VoltageSensor voltageSensor;
        private ElapsedTime timer;

        // PID variables
        private double error, error_sum, prev_error, delta_error, delta_time;
        private double output;
        private double last_time;

        // PID constants
        private double kP = 0.002;
        private double kI = 0;
        private double kD = 0.01;
        private double kF = 0.0004167;
        private double voltage_constant = 0.95;
        private double voltage_reference = 13.0;

        public double target_velocity = -1300;

        public Launcher(HardwareMap hwMap){
            launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
            launchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            launchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            launchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            voltageSensor = hwMap.get(VoltageSensor.class, "Control Hub");
            timer = new ElapsedTime();
        }

        private double computeOutput(double target){
            double current_velocity = launchMotor.getVelocity();
            double voltage = voltageSensor.getVoltage();

            double now = timer.seconds();
            delta_time = (now - last_time > 0) ? now - last_time : 1e-3;

            error = target - current_velocity;
            error_sum += error * delta_time;
            delta_error = (error - prev_error) / delta_time;

            double ffv = kF * target * (voltage_reference / voltage) * voltage_constant;

            output = ffv + (kP * error) + (kI * error_sum) + (kD * delta_error);

            prev_error = error;
            last_time = now;

            return output;
        }

        public Action shoot(double target){
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if(!initialized){
                        timer.reset();
                        last_time = timer.seconds();
                        initialized = true;
                    }

                    double power = computeOutput(target);
                    launchMotor.setPower(Math.max(-1, Math.min(1, power))); // clamp to [-1, 1]

                    double currentVel = launchMotor.getVelocity();
                    packet.put("PID target", target);
                    packet.put("PID velocity", currentVel);
                    packet.put("PID output", power);

                    // Shooter is ready when within 10 ticks/sec
                    return Math.abs(target - currentVel) > 10;
                }
            };
        }

        public Action spin(){
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if(!initialized){
                        timer.reset();
                        last_time = timer.seconds();
                        initialized = true;
                    }

                    double power = computeOutput(target_velocity);
                    launchMotor.setPower(Math.max(-1, Math.min(1, power)));

                    double currentVel = launchMotor.getVelocity();
                    packet.put("PID target", target_velocity);
                    packet.put("PID velocity", currentVel);
                    packet.put("PID output", power);

                    if(timer.seconds() > 4.0){
                        launchMotor.setPower(0);
                        return false;
                    }
                    return true;
                }
            };
        }

        public Action breakLauncher(){
            return packet -> {
                launchMotor.setPower(0);
                return false;
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
                .turn(Math.toRadians(-118))
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
