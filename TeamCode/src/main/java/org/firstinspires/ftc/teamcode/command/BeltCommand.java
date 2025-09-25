package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystem.BeltSubsyetm;

public class BeltCommand extends CommandBase {
    public BeltSubsyetm beltSubsyetm;
    public Gamepad gamepad;

    public BeltCommand(BeltSubsyetm beltSubsyetm, Gamepad g){
        this.beltSubsyetm = beltSubsyetm;
        this.gamepad = g;
        addRequirements(beltSubsyetm);
    }

    @Override
    public void execute(){
        beltSubsyetm.MoveBelt(true);
    }
}
