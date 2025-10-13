package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystem.LaunchSubsystem;
//Probably have to change this from a command base to a Parallel commandBase because launching will require parallel movemnt
public class LaunchCommand extends CommandBase {
    public LaunchSubsystem launchSubsystem;
    public Gamepad gamepad;

    public LaunchCommand(LaunchSubsystem l, Gamepad g){
        this.launchSubsystem = l;
        this.gamepad = g;

        addRequirements(l);
    }

    @Override
    public void execute(){
        launchSubsystem.shoot();
    }

    @Override
    public void end(boolean interrupted){
        launchSubsystem.stop();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
