package org.firstinspires.ftc.teamcode.AutoSystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Ramp {
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
                    rampServoOne.setPower(-speed);
                    rampServoTwo.setPower(speed);

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
