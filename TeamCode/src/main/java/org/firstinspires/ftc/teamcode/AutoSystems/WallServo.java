package org.firstinspires.ftc.teamcode.AutoSystems;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WallServo {
    private final Servo servo;

    // Standard positions (0.0 to 1.0)
    // 180 degrees is usually 1.0, 90 degrees is 0.5
    private static final double POS_0 = 0;
    private static final double POS_90 = 0.5;

    public WallServo(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "wallServo");
    }

    /**
     * Action to move the wall to 180 degrees (usually fully DOWN or OPEN)
     */
    public Action move0() {
        return packet -> {
            servo.setPosition(POS_0);
            packet.put("Wall Position", "180 Degrees");
            return false;
        };
    }

    /**
     * Action to move the wall to 90 degrees (usually HALF-WAY or CLOSED)
     */
    public Action move90() {
        return packet -> {
            servo.setPosition(POS_90);
            packet.put("Wall Position", "90 Degrees");
            return false;
        };
    }

    /**
     * Original flexible method for custom angles
     */
    public Action setAngle(double degrees) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double position = degrees / 180.0;

                // Clipping to prevent stalling and battery drain
                position = Math.max(0.0, Math.min(1.0, position));

                servo.setPosition(position);
                packet.put("WallServo Target Deg", degrees);
                return false;
            }
        };
    }
}