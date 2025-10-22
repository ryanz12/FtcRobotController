package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystem.LaunchSubsystem;

public class ShootCommand extends CommandBase {
    private LaunchSubsystem launch_subsystem;
    private Gamepad gamepad;

    public ShootCommand(LaunchSubsystem l, Gamepad g){
        this.launch_subsystem = l;
        this.gamepad = g;

        addRequirements(l);
    }

    @Override
    public void execute() {
        if (gamepad.b){
            launch_subsystem.shoot();
        }
    }

    @Override
    public void end(boolean interrupted) {
        launch_subsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
