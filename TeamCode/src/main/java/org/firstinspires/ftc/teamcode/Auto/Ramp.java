
package org.firstinspires.ftc.teamcode.Auto;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import org.jetbrains.annotations.NotNull;

public class Ramp implements Action {

    public enum Direction {
        UP, DOWN
    }

    private final CRServo servoOne;
    private final CRServo servoTwo;
    private final Direction direction;
    private final boolean stop;

    // Move ramp (UP or DOWN)
    public Ramp(CRServo servoOne, CRServo servoTwo, Direction direction) {
        this.servoOne = servoOne;
        this.servoTwo = servoTwo;
        this.direction = direction;
        this.stop = false;
    }

    // Stop ramp
    public Ramp(CRServo servoOne, CRServo servoTwo) {
        this.servoOne = servoOne;
        this.servoTwo = servoTwo;
        this.direction = null;
        this.stop = true;
    }

    @Override
    public boolean run(@NotNull TelemetryPacket packet) {
        if (stop) {
            servoOne.setPower(0);
            servoTwo.setPower(0);
        } else {
            servoOne.setPower(direction == Direction.UP ? 1 : -1);
            servoTwo.setPower(direction == Direction.UP ? -1 : 1);
        }
        return false;
    }
}
