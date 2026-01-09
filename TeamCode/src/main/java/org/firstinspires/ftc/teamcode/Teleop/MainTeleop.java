package org.firstinspires.ftc.teamcode.Teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.BeltCommand;
import org.firstinspires.ftc.teamcode.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.LaunchCommand;
import org.firstinspires.ftc.teamcode.Commands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.Subsystems.BeltSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LaunchSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSubsystem;

@TeleOp
public class MainTeleop extends CommandOpMode {
    private GamepadEx driveController, shootingController;
    private DriveSubsystem driveSubsystem;
    private LaunchSubsystem launchSubsystem;
    private CameraSubsystem cameraSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private BeltSubsystem beltSubsystem;
    private OuttakeSubsystem outtakeSubsystem;


    @Override
    public void initialize(){
        driveController = new GamepadEx(gamepad1);
        shootingController = new GamepadEx(gamepad2);

        driveSubsystem = new DriveSubsystem(hardwareMap);
        launchSubsystem = new LaunchSubsystem(hardwareMap, telemetry);
        cameraSubsystem = new CameraSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        beltSubsystem = new BeltSubsystem(hardwareMap);
        outtakeSubsystem = new OuttakeSubsystem(hardwareMap);

        driveSubsystem.setDefaultCommand(new DriveCommand(driveSubsystem, gamepad1));
        launchSubsystem.setDefaultCommand(new LaunchCommand(launchSubsystem, cameraSubsystem, shootingController, telemetry));
        intakeSubsystem.setDefaultCommand(new IntakeCommand(intakeSubsystem, shootingController));
        outtakeSubsystem.setDefaultCommand(new OuttakeCommand(outtakeSubsystem, shootingController));
        beltSubsystem.setDefaultCommand(new BeltCommand(beltSubsystem, shootingController));

    }
}
