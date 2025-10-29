package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;

public class IntakeCommand extends CommandBase {

    private final IntakeSubsystem intakeSubsystem;
    private final Gamepad gamepad;

    private boolean intakeActive = false;
    private boolean outtakeActive = false;

    private boolean lastX = false;
    private boolean lastA = false;

    public IntakeCommand(IntakeSubsystem i, Gamepad g) {
        this.intakeSubsystem = i;
        this.gamepad = g;

        addRequirements(i);
    }

    @Override
    public void execute() {
        boolean currentX = gamepad.x;
        boolean currentA = gamepad.a;

        if (currentX && !lastX) {
            intakeActive = !intakeActive;
            outtakeActive = false;
        }

        if (currentA && !lastA) {
            outtakeActive = !outtakeActive;
            intakeActive = false;
        }

        lastX = currentX;
        lastA = currentA;

        if (intakeActive) {
            intakeSubsystem.intake();
        } else if (outtakeActive) {
            intakeSubsystem.outake();
        } else {
            intakeSubsystem.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
        intakeActive = false;
        outtakeActive = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
