package org.firstinspires.ftc.teamcode.pedroPathing.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystem.DriveSubsystem;

public class DriveCommand extends CommandBase {
    private DriveSubsystem driveSubsystem;
    private Gamepad gamepad;

    public DriveCommand(DriveSubsystem driveSubsystem, Gamepad gamepad){
        // decoupling code BITCH
        this.driveSubsystem = driveSubsystem;
        this.gamepad = gamepad;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        // use the Follower within the DriveSubsystem to handle driving
        double forward = gamepad.left_stick_y;
        double strafe = -gamepad.left_stick_x;
        double rotate = -gamepad.right_stick_x;

        driveSubsystem.drive(forward, strafe, rotate);

        if (gamepad.dpad_left){
            driveSubsystem.drive(0, 0.85, 0);
        }

        if (gamepad.dpad_right){
            driveSubsystem.drive(0, -0.85, 0);
        }

        if (gamepad.dpad_up){
            driveSubsystem.drive(-0.8, 0, 0);
        }

        if (gamepad.dpad_down){
            driveSubsystem.drive(0.8, 0, 0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;  // runs continuously until interrupted
    }
}
