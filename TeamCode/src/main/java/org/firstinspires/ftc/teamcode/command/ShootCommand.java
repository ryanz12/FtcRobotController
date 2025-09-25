package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystem.BeltSubsyetm;
import org.firstinspires.ftc.teamcode.subsystem.LaunchSubsystem;

public class ShootCommand  extends SequentialCommandGroup {
    private Gamepad gamepad;
    public ShootCommand(LaunchSubsystem launchSubsystem, BeltSubsyetm beltSubsyetm, Gamepad gamepadEx){
        addCommands(
                new BeltCommand(beltSubsyetm,gamepadEx),
                new LaunchCommand(launchSubsystem,gamepadEx)
        );
    }
}
