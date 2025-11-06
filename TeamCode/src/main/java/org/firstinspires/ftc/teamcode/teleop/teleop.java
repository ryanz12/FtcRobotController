package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.command.BeltCommand;
import org.firstinspires.ftc.teamcode.command.DriveCommand;
import org.firstinspires.ftc.teamcode.command.IntakeCommand;
import org.firstinspires.ftc.teamcode.command.ShootCommand;
import org.firstinspires.ftc.teamcode.subsystem.BeltSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LaunchSubsystem;

@TeleOp
public class teleop extends CommandOpMode {
    //Buttons
    private com.arcrobotics.ftclib.command.button.Button shootButton;
    private GamepadEx shooting_controller, drive_controller;
    private DriveSubsystem drive_subsystem;

    private LaunchSubsystem launchSubsystem;
    private BeltSubsystem beltSubsystem;

    private IntakeSubsystem intakeSubsystem;

    private Button shoot_button, belt_button, intake_button;

    // TODO:
    // Double check shoot button and related
    // Need to adjust the outake to stop when power is off

    @Override
    public void initialize() {
        drive_controller = new GamepadEx(gamepad1);
        shooting_controller = new GamepadEx(gamepad2);

        launchSubsystem = new LaunchSubsystem(hardwareMap);
        beltSubsystem = new BeltSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        drive_subsystem = new DriveSubsystem(hardwareMap);

        drive_subsystem.setDefaultCommand(new DriveCommand(drive_subsystem, gamepad1));
        intakeSubsystem.setDefaultCommand(new IntakeCommand(intakeSubsystem, shooting_controller));

        new GamepadButton(shooting_controller)
                .whenPressed(new BeltCommand(beltSubsystem, gamepad2));
        //Maybe we should make this toggle as well.
        new GamepadButton(shooting_controller, GamepadKeys.Button.B)
                .whenPressed(new ShootCommand(launchSubsystem,gamepad2));
    }
}
