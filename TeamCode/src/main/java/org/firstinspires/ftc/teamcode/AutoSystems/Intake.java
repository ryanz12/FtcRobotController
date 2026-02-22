package org.firstinspires.ftc.teamcode.AutoSystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {
    private DcMotorEx intakeMotor;
    private ElapsedTime timer;

    public Intake(HardwareMap hwMap){
        intakeMotor = hwMap.get(DcMotorEx.class, "intakeMotor");
        timer = new ElapsedTime();
    }

    public Action roll(double seconds, double power){
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized){
                    intakeMotor.setPower(-power);
                    initialized = true;
                    timer.reset();
                }

                if (timer.seconds() > seconds){
                    intakeMotor.setPower(0);
                    return false;
                }

                return true;
            }
        };
    }
    public Action dynamicRoll(double seconds, double initialPower) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    timer.reset();
                    initialized = true;
                }

                double elapsed = timer.seconds();

                if (elapsed < seconds) {
                    // Calculate the multiplier (goes from 1.0 down to 0.0)
                    double factor = 1.8 - (elapsed / seconds);

                    // Scale the initial power by the factor
                    double currentPower = initialPower * factor;

                    intakeMotor.setPower(-currentPower);

                    // Log to dashboard so you can see the curve
                    telemetryPacket.put("Intake Power", currentPower);
                    return true;
                } else {
                    intakeMotor.setPower(0);
                    return false;
                }
            }
        };
    }
    public Action rampUpRoll(double seconds, double minPower, double targetPower) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    timer.reset();
                    initialized = true;
                }

                double elapsed = timer.seconds();

                if (elapsed < seconds) {
                    // Calculate progress (0.0 at start, 1.0 at end)
                    double progress = elapsed / seconds;

                    // Linear interpolation: minPower + (difference * progress)
                    double currentPower = minPower + (targetPower - minPower) * progress;

                    intakeMotor.setPower(-currentPower);

                    telemetryPacket.put("Intake Ramp Power", currentPower);
                    return true;
                } else {
                    intakeMotor.setPower(0);
                    return false;
                }
            }
        };
    }
    public void setPower(double power) {
        intakeMotor.setPower(-power); // Kept your negative sign from the Action
    }
}