package org.firstinspires.ftc.teamcode.lab;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.command.TrackCommand;
import org.firstinspires.ftc.teamcode.subsystem.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LaunchSubsystem;

@TeleOp
public class TrackingTest extends CommandOpMode {
    private DriveSubsystem driveSubsystem;
    private CameraSubsystem cameraSubsystem;

    private GamepadEx driveController;

    private boolean cameraOn = false;
    private Command trackCommand;

    @Override
    public void initialize(){
        driveSubsystem = new DriveSubsystem(hardwareMap);
        if (cameraSubsystem == null) cameraSubsystem = new CameraSubsystem(hardwareMap, telemetry);

        driveController = new GamepadEx(gamepad1);

        trackCommand = new TrackCommand(cameraSubsystem, driveSubsystem, telemetry);

        new GamepadButton(driveController, GamepadKeys.Button.X)
                .toggleWhenPressed(trackCommand);

    }


}
