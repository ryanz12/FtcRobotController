package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSubsystem;

public class OuttakeCommand extends CommandBase {
    private OuttakeSubsystem OuttakeSubsystem;
    private GamepadEx gamepad;
    public OuttakeCommand(OuttakeSubsystem OuttakeSubsystem, GamepadEx g){
        this.OuttakeSubsystem = OuttakeSubsystem;
        this.gamepad = g;

        addRequirements(OuttakeSubsystem);
    }

    @Override
    public void execute() {
        if (gamepad.getButton(GamepadKeys.Button.DPAD_LEFT)){
            OuttakeSubsystem.intake();
        }
        else if (gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT)){
            OuttakeSubsystem.outtake();
        }
        else {
            OuttakeSubsystem.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        OuttakeSubsystem.stop();

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
