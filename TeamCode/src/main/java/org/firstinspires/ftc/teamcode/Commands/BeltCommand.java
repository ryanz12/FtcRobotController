package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Subsystems.BeltSubsystem;

public class BeltCommand extends CommandBase {
    private BeltSubsystem beltSubsystem;
    private BeltSubsystem.Direction direction;
    private GamepadEx gamepad;
    public BeltCommand(BeltSubsystem beltSubsystem, GamepadEx g){
        this.beltSubsystem = beltSubsystem;
        this.gamepad = g;

        addRequirements(beltSubsystem);
    }

    @Override
    public void execute() {
        if (gamepad.getButton(GamepadKeys.Button.Y)){
            beltSubsystem.move_belt(BeltSubsystem.Direction.UTR);
        }
        else if (gamepad.getButton(GamepadKeys.Button.X)){
            beltSubsystem.move_belt(BeltSubsystem.Direction.DTR);
        }
        else {
            beltSubsystem.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        beltSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
