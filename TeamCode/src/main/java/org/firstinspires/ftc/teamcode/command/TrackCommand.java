package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LaunchSubsystem;

import java.util.List;

public class TrackCommand extends CommandBase {

    private CameraSubsystem cameraSubsystem;
    private DriveSubsystem driveSubsystem;

    public TrackCommand(CameraSubsystem cameraSubsystem, DriveSubsystem driveSubsystem, Telemetry telemetry){
        this.cameraSubsystem = cameraSubsystem;
        this.driveSubsystem = driveSubsystem;

        addRequirements(cameraSubsystem, driveSubsystem);
    }

    @Override
    public void initialize(){
        cameraSubsystem.startStreaming();
    }

    @Override
    public void execute(){
        // Is it gonna be more efficient to close it everytime I toggle or pause it and background run? nvm I dont think it closes
        // +ve need to go right, -ve need to go left
        List<Double> cameraData = cameraSubsystem.getCameraData();

        double x, range, turn, power;

        if (cameraData == null){
            x = range = 0;
        }
        else {
            x = cameraData.get(0);
            range = cameraData.get(1);
        }

        if (x > 2){
            turn = -0.2;
        }
        else if (x < -2){
            turn = 0.2;
        }
        else {
            turn = 0;
        }

        driveSubsystem.drive(0, 0, turn);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0,0, 0);
        cameraSubsystem.stopStreaming();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
