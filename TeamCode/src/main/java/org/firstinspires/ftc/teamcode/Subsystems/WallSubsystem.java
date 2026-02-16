package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WallSubsystem extends SubsystemBase {
    private final Servo wallServo;

    // Define the positions clearly
    public enum WallPosition {
        DOWN(1.0), // 180 degrees
        UP(0.5);   // 90 degrees

        public final double position;
        WallPosition(double position) {
            this.position = position;
        }
    }

    public WallSubsystem(HardwareMap hMap) {
        wallServo = hMap.get(Servo.class, "wallServo");
        // Apply your reverse direction here
        wallServo.setDirection(Servo.Direction.REVERSE);
    }

    /**
     * TeleOp Method: Sets the servo position based on the Enum
     */
    public void setPosition(WallPosition target) {
        wallServo.setPosition(target.position);
    }

    /**
     * TeleOp Method: Directly gets current position for telemetry
     */
    public double getCurrentPosition() {
        return wallServo.getPosition();
    }

    // --- Road Runner Actions ---

    /**
     * Autonomous Action: Moves the wall to a specific position
     */
    public Action moveAction(WallPosition target) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wallServo.setPosition(target.position);
                packet.put("Wall Target", target.name());
                return false;
            }
        };
    }
}