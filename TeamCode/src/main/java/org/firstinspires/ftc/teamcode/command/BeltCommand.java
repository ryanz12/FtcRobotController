package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystem.BeltSubsystem;

public class BeltCommand extends CommandBase {

    private BeltSubsystem belt_subsystem;
    private Gamepad gamepad;

    public BeltCommand(BeltSubsystem b, Gamepad g){
        this.belt_subsystem = b;
        this.gamepad = g;

        addRequirements(b);
    }

    @Override
    public void execute() {
        if (gamepad.a){
            belt_subsystem.move_belt(BeltSubsystem.Direction.UTR);
        }
    }

    @Override
    public void end(boolean interrupted){
        belt_subsystem.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
