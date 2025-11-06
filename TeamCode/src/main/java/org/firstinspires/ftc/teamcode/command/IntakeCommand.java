package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;

public class IntakeCommand extends CommandBase {

    private final IntakeSubsystem intakeSubsystem;
    private final GamepadEx gamepad;


    public IntakeCommand(IntakeSubsystem i, GamepadEx g) {
        this.intakeSubsystem = i;
        this.gamepad = g;

        addRequirements(i);
    }

    @Override
    public void execute() {
        if (gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)){
            intakeSubsystem.intake();
        }
        else if (gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)){
            intakeSubsystem.outtake();
        }
        else {
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
