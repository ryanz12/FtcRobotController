package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;

public class IntakeCommand extends CommandBase {

    private final IntakeSubsystem intakeSubsystem;
    private final Gamepad gamepad;

    public IntakeCommand(IntakeSubsystem i, Gamepad g) {
        this.intakeSubsystem = i;
        this.gamepad = g;

        addRequirements(i);
    }

    @Override
    public void execute() {
        if (gamepad.x) {
            intakeSubsystem.intake();
        } else if (gamepad.a) {
            intakeSubsystem.outake();
        } else {
            intakeSubsystem.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
