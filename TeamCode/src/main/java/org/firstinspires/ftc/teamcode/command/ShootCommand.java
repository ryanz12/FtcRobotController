package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystem.LaunchSubsystem;

public class ShootCommand extends CommandBase {
    private final LaunchSubsystem launchSubsystem;
    private final Gamepad gamepad;

    public ShootCommand(LaunchSubsystem launchSubsystem, Gamepad gamepad) {
        this.launchSubsystem = launchSubsystem;
        this.gamepad = gamepad;
        addRequirements(launchSubsystem);
    }

    @Override
    public void execute() {
        launchSubsystem.shoot();
    }

    @Override
    public void end(boolean interrupted) {
        launchSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}