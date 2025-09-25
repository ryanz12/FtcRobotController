package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.command.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;

public class teleop extends CommandOpMode {
    //Buttons
    private com.arcrobotics.ftclib.command.button.Button shootButton;
    private GamepadEx shooting_controller, drive_controller;
    private DriveSubsystem drive_subsystem;

    @Override
    public void initialize() {
        drive_controller = new GamepadEx(gamepad1);
        shooting_controller = new GamepadEx(gamepad2);

        drive_subsystem = new DriveSubsystem(hardwareMap);
        drive_subsystem.setDefaultCommand(new DriveCommand(drive_subsystem, gamepad1));
    }
}
