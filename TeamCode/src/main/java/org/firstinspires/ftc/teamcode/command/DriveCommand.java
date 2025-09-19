package org.firstinspires.ftc.teamcode.command;


import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;

public class DriveCommand extends CommandBase {
    public DriveSubsystem drive_subsystem;
    public Gamepad gamepad;

    public DriveCommand(DriveSubsystem d, Gamepad g){
        this.drive_subsystem = d;
        this.gamepad = g;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive_subsystem);
    }

    @Override
    public void execute(){
        double forward = gamepad.left_stick_y;
        double strafe = -gamepad.left_stick_x;
        double turn = -gamepad.right_stick_x;

        drive_subsystem.drive(strafe, forward, turn);
    }

    @Override
    public boolean isFinished(){
        // Never finish because why would you stop driving
        return false;
    }

}
