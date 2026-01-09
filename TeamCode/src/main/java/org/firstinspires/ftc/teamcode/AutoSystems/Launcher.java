package org.firstinspires.ftc.teamcode.AutoSystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Launcher {
    private DcMotorEx launchMotor;
    private ElapsedTime timer;
    private double currentVelocity;
    private double targetVelocity;

    private double kP = 0.003;
    private double kI = 0;
    private double kD = 0.01;
    private double kF = 0.0004167;
    private double error, errorSum, prevError, de, dt;
    private double output;

    public Launcher (HardwareMap hwMap){
        launchMotor = hwMap.get(DcMotorEx.class, "launchMotor");
        launchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        launchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        timer = new ElapsedTime();
    }

    private double calcPower(double t){
        currentVelocity = launchMotor.getVelocity();

        error = targetVelocity-currentVelocity;
        errorSum += error;

        dt = t > 0 ? t : 1e-3;
        de = (error-prevError)/dt;
        prevError = error;

        double ff = kF * targetVelocity;

        output = ff + kP * error + kI * errorSum + kD * de;

        return -output;
    }

    public Action shoot(double target, double time){
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized){
                    timer.reset();
                    targetVelocity = target;
                    initialized = true;
                }

                double currentTime = timer.seconds();

                if (currentTime >= time){
                    launchMotor.setPower(0);
                    error = errorSum = prevError = de = dt = 0;

                    return false;
                }

                double power = calcPower(currentTime);
                launchMotor.setPower(power);

                telemetryPacket.put("current velocity", launchMotor.getVelocity());
                telemetryPacket.put("target velocity", targetVelocity);
                telemetryPacket.put("power", power);
                telemetryPacket.put("time", timer.seconds());

                return true;
            }
        };
    }
}
