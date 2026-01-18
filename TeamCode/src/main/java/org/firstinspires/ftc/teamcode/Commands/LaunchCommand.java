package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LaunchSubsystem;

import java.util.List;

public class LaunchCommand extends CommandBase {
    private LaunchSubsystem launchSubsystem;
    private GamepadEx gamepad1, gamepad2;
    private Telemetry telemetry;

    enum MODE {
        ON,
        OFF
    }

    enum RANGE {
        FAR,
        CLOSE
    }


    private MODE state = MODE.OFF;
    private RANGE r = RANGE.CLOSE;

    public LaunchCommand(LaunchSubsystem launchSubsystem, GamepadEx gamepad2, GamepadEx gamepad1, Telemetry telemetry){
        this.launchSubsystem = launchSubsystem;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;

        addRequirements(launchSubsystem);
    }

    public double calculateVelocity(double dx){
        double vi;
        double dy = 0.855;
        double ay = -9.8;
        double theta = Math.toRadians(45);

        vi = Math.sqrt(ay*Math.pow(dx, 2)/(0.855-dx));
        telemetry.addData("Vel", ay*Math.pow(dx, 2)/(0.855-dx));
        return vi;
    }

    @Override
    public void execute(){
        if (gamepad1.getButton(GamepadKeys.Button.B)){
            state = MODE.ON;
        }
        if (gamepad1.getButton(GamepadKeys.Button.A)){
            state = MODE.OFF;
        }

//        List<Double> cameraData = cameraSubsystem.getCameraData();
//        double range;
//
//        if (cameraData == null){
//            range = 0;
//        }
//        else {
//            range = cameraData.get(1);
//        }

        if (state == MODE.ON){
//            double targetSpeed = speedFromDistanceCm(range);  // range assumed in cm

            if (gamepad2.getButton(GamepadKeys.Button.A)){
                r = RANGE.CLOSE;
            }
            else if (gamepad2.getButton(GamepadKeys.Button.B)){
                r = RANGE.FAR;
            }

            if (r == RANGE.CLOSE){
                launchSubsystem.setSpeed(-1250, telemetry);
            }
            else if (r == RANGE.FAR){
                launchSubsystem.setSpeed(-1500, telemetry);
            }

//            telemetry.addData("April Tag distance:", range);
            telemetry.update();
        }
        else {
            launchSubsystem.target = 0;
        }

    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        launchSubsystem.stop();
    }
}
