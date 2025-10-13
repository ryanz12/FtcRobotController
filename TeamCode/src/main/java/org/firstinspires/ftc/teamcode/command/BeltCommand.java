package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystem.BeltSubsystem;

public class BeltCommand extends CommandBase {
    public BeltSubsystem beltSubsystem;

    // Having a gamepad in here has no use
    public Gamepad gamepad;

    public BeltCommand(BeltSubsystem beltSubsystem, Gamepad g){
        this.beltSubsystem = beltSubsystem;
        this.gamepad = g;
        addRequirements(beltSubsystem);
    }

    @Override
    public void execute(){
        // Wait... when are you not moving the ramp up?
        beltSubsystem.move_belt(BeltSubsystem.Direction.UTR);
    }
}
