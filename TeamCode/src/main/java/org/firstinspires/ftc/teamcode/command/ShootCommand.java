package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystem.LaunchSubsystem;

public class ShootCommand extends CommandBase {

    //TODO: implement periodic function for subsystem instead of doing ts

    private LaunchSubsystem launchSubsystem;
    private GamepadEx gamepad;


    private boolean pastBPress = false;
    private boolean pastAPress = false;

    private boolean shootForward = false;
    private boolean shootReverse = false;

    public ShootCommand(LaunchSubsystem launchSubsystem, GamepadEx gamepad){
        this.launchSubsystem = launchSubsystem;
        this.gamepad = gamepad;

        addRequirements(launchSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        boolean currBPress = gamepad.getButton(GamepadKeys.Button.B);
        boolean currAPress = gamepad.getButton(GamepadKeys.Button.A);

        // Edge detection
        boolean bPressed = currBPress && !pastBPress;
        boolean aPressed = currAPress && !pastAPress;

        // --- Toggle forward shooting with B ---
        if (bPressed) {
            shootForward = !shootForward;
            if (shootForward) {
                shootReverse = false;
            }
        }

        // --- Toggle reverse shooting with A ---
        if (aPressed) {
            shootReverse = !shootReverse;
            if (shootReverse) {
                shootForward = false;
            }
        }

        if (shootForward) {
            launchSubsystem.set_pid(-1200);
        } else if (shootReverse) {
            launchSubsystem.set_pid(2400);
        } else {
            launchSubsystem.set_pid(0);
        }

        pastBPress = currBPress;
        pastAPress = currAPress;
    }


    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shootForward = false;
        shootReverse = false;
        launchSubsystem.stop();
    }
}