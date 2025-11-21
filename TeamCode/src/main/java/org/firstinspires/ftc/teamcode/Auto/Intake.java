package org.firstinspires.ftc.teamcode.Auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake implements Action {
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
        intakeServo.setPower(direction == 1 ? 1 : -0.5);

        return timer.seconds() < 1 ? true : false;
    }
}