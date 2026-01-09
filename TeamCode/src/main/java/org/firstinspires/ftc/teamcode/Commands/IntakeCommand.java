package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
    private IntakeSubsystem intakeSubsystem;
    private GamepadEx gamepad;
    public IntakeCommand(IntakeSubsystem intakeSubsystem, GamepadEx g){
        this.intakeSubsystem = intakeSubsystem;
        this.gamepad = g;

        addRequirements(intakeSubsystem);
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
