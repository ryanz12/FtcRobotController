package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystem.LaunchSubsystem;

public class ShootCommand extends CommandBase {
    private final LaunchSubsystem launchSubsystem;
    private final Gamepad gamepad;
    private boolean pastButtonPress = false;
    private boolean shoot = false;

    public ShootCommand(LaunchSubsystem launchSubsystem, Gamepad gamepad) {
        this.launchSubsystem = launchSubsystem;
        this.gamepad = gamepad;
        addRequirements(launchSubsystem);
    }

    @Override
    public void execute() {
        boolean currButtonPress = gamepad.b;

        if(currButtonPress && !pastButtonPress) {
            shoot = !shoot;
            if(shoot) launchSubsystem.shoot();
            else launchSubsystem.stop();
        }

        pastButtonPress = currButtonPress;
    }

    @Override
    public boolean isFinished() {
        return false; // keeps checking for input
    }

    @Override
    public void end(boolean interrupted) {
        launchSubsystem.stop();
    }

}