package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystem.BeltSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LaunchSubsystem;


public class ShootCommand extends ParallelCommandGroup {

    // Wouldn't you want to use a ParallelCommandGroup instead of Sequential?
    // Feel like that would be more productive

    private Gamepad gamepad;
    public ShootCommand(LaunchSubsystem launchSubsystem, BeltSubsystem beltSubsystem, Gamepad gamepadEx){
        addCommands(
                new BeltCommand(beltSubsystem, gamepadEx),
                new LaunchCommand(launchSubsystem, gamepadEx)
        );

        addRequirements(launchSubsystem, beltSubsystem);
    }
}
