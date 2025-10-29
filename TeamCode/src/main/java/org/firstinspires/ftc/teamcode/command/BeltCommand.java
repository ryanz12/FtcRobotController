package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystem.BeltSubsystem;

public class BeltCommand extends CommandBase {

    private final BeltSubsystem beltSubsystem;
    private boolean movingUp = true; // track current direction
    private boolean isRunning = false;

    public BeltCommand(BeltSubsystem beltSubsystem) {
        this.beltSubsystem = beltSubsystem;
        addRequirements(beltSubsystem);
    }

    @Override
    public void initialize() {
        if (!isRunning) {
            beltSubsystem.move_belt(movingUp ? BeltSubsystem.Direction.UTR : BeltSubsystem.Direction.DTR);
            movingUp = !movingUp;
            isRunning = true;
        } else {
            beltSubsystem.stop();
            isRunning = false;
        }
    }

    @Override
    public boolean isFinished() {
        return true; // ends instantly after toggling
    }

}
