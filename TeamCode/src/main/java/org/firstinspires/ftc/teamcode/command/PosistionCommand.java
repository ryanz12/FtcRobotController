package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;

public class PosistionCommand extends CommandBase {
    //TODO
    //Step 1
    //      Pause the Webcam 1 view port
    //Step 2
    //      Switch to camera 2 use its viewport and Camera detection
    //      Use the distance between the camera 2 and april tags.
    //      Position the robot the correct heading and distance

    private boolean viewportPaused = false;
    private  final BNO055IMU imu;

    private final Gamepad gamepad;
    private DriveSubsystem driveSubsystem;

    private PosistionCommand (BNO055IMU imu, DriveSubsystem driveSubsystem, Gamepad gamepad){
        this.imu = imu;
        this.driveSubsystem = driveSubsystem;
        this.gamepad = gamepad;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute(){
        if(gamepad.a){
            //
        }
    }

    public void end(boolean interrupted) {

    }
}
