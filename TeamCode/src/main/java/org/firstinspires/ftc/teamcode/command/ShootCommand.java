package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystem.LaunchSubsystem;

public class ShootCommand extends CommandBase {

    private final LaunchSubsystem launchSubsystem;
    private final Gamepad gamepad;

    private boolean pastBPress = false;
    private boolean pastAPress = false;

    private boolean shootForward = false;
    private boolean shootReverse = false;

    public ShootCommand(LaunchSubsystem launchSubsystem, Gamepad gamepad) {
        this.launchSubsystem = launchSubsystem;
        this.gamepad = gamepad;
        addRequirements(launchSubsystem);
    }

    @Override
    public void execute() {
        boolean currBPress = gamepad.b;
        boolean currAPress = gamepad.a;

        // --- Toggle forward shooting with B ---
        if (currBPress && !pastBPress) {
            shootForward = !shootForward;
            if (shootForward) {
                shootReverse = false;  // stop reverse if active
                launchSubsystem.shoot(0.6);
            } else {
                launchSubsystem.stop();
            }
        }

        // --- Toggle reverse shooting with A ---
        if (currAPress && !pastAPress) {
            shootReverse = !shootReverse;
            if (shootReverse) {
                shootForward = false;  // stop forward if active
                launchSubsystem.reverseShoot();
            } else {
                launchSubsystem.stop();
            }
        }

        pastBPress = currBPress;
        pastAPress = currAPress;
    }

    @Override
    public boolean isFinished() {
        return false; // continuously checks input
    }

    @Override
    public void end(boolean interrupted) {
        launchSubsystem.stop();
    }
}
