package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.subsystem.BeltSubsystem;

public class BeltCommand extends CommandBase {

    private final BeltSubsystem beltSubsystem;
    private final Gamepad gamepad;

    private boolean isMovingUp = false;
    private boolean isMovingDown = false;

    // Debounce tracking (to prevent rapid toggling while button is held)
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;

    public BeltCommand(BeltSubsystem beltSubsystem, Gamepad gamepad) {
        this.beltSubsystem = beltSubsystem;
        this.gamepad = gamepad;
        addRequirements(beltSubsystem);
    }

    @Override
    public void execute() {
        boolean dpadUp = gamepad.left_bumper;
        boolean dpadDown = gamepad.right_bumper;

        // ---- D-PAD UP ----
        if (dpadUp && !prevDpadUp) { // on press (not hold)
            if (!isMovingUp) {
                beltSubsystem.move_belt(BeltSubsystem.Direction.UTR);
                isMovingUp = true;
                isMovingDown = false;
            } else {
                beltSubsystem.stop();
                isMovingUp = false;
            }
        }

        // ---- D-PAD DOWN ----
        if (dpadDown && !prevDpadDown) { // on press (not hold)
            if (!isMovingDown) {
                beltSubsystem.move_belt(BeltSubsystem.Direction.DTR);
                isMovingDown = true;
                isMovingUp = false;
            } else {
                beltSubsystem.stop();
                isMovingDown = false;
            }
        }

        // Update previous states for debounce
        prevDpadUp = dpadUp;
        prevDpadDown = dpadDown;
    }

    @Override
    public boolean isFinished() {
        return false; // keeps checking for input
    }

    @Override
    public void end(boolean interrupted) {
        beltSubsystem.stop();
    }
}
