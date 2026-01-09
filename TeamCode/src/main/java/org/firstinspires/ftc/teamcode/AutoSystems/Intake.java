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
}