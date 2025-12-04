package org.firstinspires.ftc.teamcode.command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;

public class DriveCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final Gamepad gamepad;
    private enum DriveState {
        NORMAL,
        INVERTED
    }
    private DriveState currentState = DriveState.NORMAL;
    private boolean yPressedLast = false;
    public DriveCommand(DriveSubsystem d, Gamepad g) {
        this.driveSubsystem = d;
        this.gamepad = g;
        addRequirements(driveSubsystem);
    }
    @Override
    public void execute() {
        boolean yPressedNow = gamepad.y;
        if (yPressedNow && !yPressedLast) {
            toggleDriveState();
        }
        yPressedLast = yPressedNow;
        if (currentState == DriveState.NORMAL) {
            normalControls();
        } else {
            invertedControls();
        }
    }
    private void toggleDriveState() {
        if (currentState == DriveState.NORMAL) {
            currentState = DriveState.INVERTED;
        } else {
            currentState = DriveState.NORMAL;
        }
    }
    private void normalControls() {
        double forward = gamepad.left_stick_y;
        double strafe = -gamepad.left_stick_x;
        double turn = -gamepad.right_stick_x;
        driveSubsystem.drive(strafe, forward, turn);
    }
    private void invertedControls() {
        double forward = -gamepad.left_stick_y;
        double strafe = gamepad.left_stick_x;
        double turn = - gamepad.right_stick_x;
        driveSubsystem.drive(strafe, forward, turn);
    }
    @Override
    public boolean isFinished() {
        // Never finish driving
        return false;
    }
}
