package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Subsystems.WallSubsystem;

public class WallCommand extends CommandBase {
    private final WallSubsystem wallSubsystem;
    private final GamepadEx gamepad;

    public WallCommand(WallSubsystem wallSubsystem, GamepadEx g) {
        this.wallSubsystem = wallSubsystem;
        this.gamepad = g;

        // Requirements prevent multiple commands from using the same subsystem at once
        addRequirements(wallSubsystem);
    }

    @Override
    public void execute() {
        if (gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
            // Move to 180 degrees
            wallSubsystem.setPosition(WallSubsystem.WallPosition.DOWN);
        }
        else if (gamepad.getButton(GamepadKeys.Button.DPAD_LEFT)) {
            // Move to 90 degrees
            wallSubsystem.setPosition(WallSubsystem.WallPosition.UP);
        }
        // Note: We don't call a "stop" here because a standard Servo
        // holds its position automatically once set.
    }

    @Override
    public boolean isFinished() {
        // Returning false keeps the command active so it can listen for button presses
        return false;
    }
}