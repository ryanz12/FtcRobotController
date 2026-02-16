package org.firstinspires.ftc.teamcode.AutoSystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HoldSystem {
    private CRServo holdServo;
    private ElapsedTime timer;

    public HoldSystem(HardwareMap hwMap){
        holdServo = hwMap.get(CRServo.class, "outtakeServo");
        timer = new ElapsedTime();
    }
    public Action hold(double seconds, double speed){
        return new Action(){
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!initialized){
                    timer.reset();
                    holdServo.setPower(speed);

                    initialized = true;
                }

                telemetryPacket.put("running true", true);

                if (timer.seconds() > seconds){
                    holdServo.setPower(0);
                    return false;
                }

                return true;
            }
        };
    }
    public Action release(double seconds, double speed){
        return new Action(){
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!initialized){
                    timer.reset();
                    holdServo.setPower(-speed);

                    initialized = true;
                }

                telemetryPacket.put("running true", true);

                if (timer.seconds() > seconds){
                    holdServo.setPower(0);
                    return false;
                }

                return true;
            }
        };
    }
    public void setPower(double speed) {
        holdServo.setPower(speed);
    }

}
