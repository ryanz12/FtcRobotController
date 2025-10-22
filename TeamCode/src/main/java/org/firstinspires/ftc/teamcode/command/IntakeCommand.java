package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.button.Button;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;

public class IntakeCommand extends CommandBase {

    private IntakeSubsystem intakeSubsystem;

    private Gamepad gamepad;


    public IntakeCommand(IntakeSubsystem i, Gamepad g) {
        this.intakeSubsystem = i;
        this.gamepad = g;

        addRequirements(i);
    }

    @Override
    public void execute(){
        if (gamepad.x){
            intakeSubsystem.intake();
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
    public boolean isFinished(){
        return false;
    }
}
