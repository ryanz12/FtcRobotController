package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.subsystem.BeltSubsystem;

public class BeltCommand extends CommandBase {
    private final BeltSubsystem beltSubsystem;
    private final BeltSubsystem.Direction direction;

    public BeltCommand(BeltSubsystem beltSubsystem, Gamepad gamepad, BeltSubsystem.Direction direction) {
        this.beltSubsystem = beltSubsystem;
        this.direction = direction;
        addRequirements(beltSubsystem);
    }

    @Override
    public void initialize() {
        beltSubsystem.move_belt(direction);
    }

    @Override
    public boolean isFinished() {
        return false; // keeps running until manually interrupted
    }

    @Override
    public void end(boolean interrupted) {
        beltSubsystem.stop();
    }
}
