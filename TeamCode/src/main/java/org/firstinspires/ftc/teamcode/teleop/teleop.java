package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.command.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;

public class teleop extends CommandOpMode {
    private DriveSubsystem drive_subsystem;

    @Override
    public void initialize() {

        drive_subsystem = new DriveSubsystem(hardwareMap);
        drive_subsystem.setDefaultCommand(new DriveCommand(drive_subsystem, gamepad1));

    }
}
