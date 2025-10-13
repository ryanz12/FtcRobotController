package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.command.DriveCommand;
import org.firstinspires.ftc.teamcode.command.ShootCommand;
import org.firstinspires.ftc.teamcode.subsystem.BeltSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LaunchSubsystem;

public class teleop extends CommandOpMode {
    //Buttons
    private com.arcrobotics.ftclib.command.button.Button shootButton;
    private GamepadEx shooting_controller, drive_controller;
    private DriveSubsystem drive_subsystem;

    private LaunchSubsystem launchSubsystem;
    private BeltSubsystem beltSubsystem;

    private Button shoot_button;

    @Override
    public void initialize() {
        drive_controller = new GamepadEx(gamepad1);
        shooting_controller = new GamepadEx(gamepad2);

        drive_subsystem = new DriveSubsystem(hardwareMap);
        drive_subsystem.setDefaultCommand(new DriveCommand(drive_subsystem, gamepad1));

        shoot_button = (new GamepadButton(shooting_controller, GamepadKeys.Button.X))
                .whenHeld(new ShootCommand(launchSubsystem, beltSubsystem, gamepad2));
    }
}
