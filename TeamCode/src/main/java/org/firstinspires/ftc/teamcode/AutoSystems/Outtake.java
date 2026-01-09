package org.firstinspires.ftc.teamcode.AutoSystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Outtake {
    private CRServo intakeMotor;
    private ElapsedTime timer;

    public Outtake(HardwareMap hwMap){
        intakeMotor = hwMap.get(CRServo.class, "outtakeServo");
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
}