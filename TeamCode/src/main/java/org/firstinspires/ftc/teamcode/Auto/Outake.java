package org.firstinspires.ftc.teamcode.Auto;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Outake implements Action {
    private final DcMotor launchMotor;
    private final boolean doStart;

    public Outake(DcMotor motor, boolean doStart) {
        this.launchMotor = motor;
        this.doStart = doStart;
    }
    private void startLaunch() {
        launchMotor.setPower(0.6);
    }

    private void stopLaunch() {
        launchMotor.setPower(0.0);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (doStart) {
            startLaunch();
        } else {
            stopLaunch();
        }
        return false;
    }
}