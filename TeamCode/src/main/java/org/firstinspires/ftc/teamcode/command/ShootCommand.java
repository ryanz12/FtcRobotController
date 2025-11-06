package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystem.LaunchSubsystem;

public class ShootCommand extends CommandBase {
    private final LaunchSubsystem launchSubsystem;
    private final Gamepad gamepad;

    private boolean toSpin = false;

    public ShootCommand(LaunchSubsystem launchSubsystem, Gamepad gamepad) {
        this.launchSubsystem = launchSubsystem;
        this.gamepad = gamepad;
        addRequirements(launchSubsystem);
    }

    @Override
    public void execute() {
        toSpin = !toSpin;

        if(toSpin) launchSubsystem.shoot();
        else launchSubsystem.stop();
    }

//    @Override
//    public void end(boolean interrupted) {
//        launchSubsystem.stop();
//    }

}