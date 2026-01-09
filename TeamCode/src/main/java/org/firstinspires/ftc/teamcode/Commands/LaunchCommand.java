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
    private CameraSubsystem cameraSubsystem;
    private GamepadEx gamepad;
    private Telemetry telemetry;

    enum MODE {
        ON,
        OFF
    }

    private MODE state = MODE.OFF;

    public LaunchCommand(LaunchSubsystem launchSubsystem, CameraSubsystem cameraSubsystem, GamepadEx gamepad, Telemetry telemetry){
        this.launchSubsystem = launchSubsystem;
        this.cameraSubsystem = cameraSubsystem;
        this.gamepad = gamepad;
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

    private static double clamp(double x, double lo, double hi) {
        return Math.max(lo, Math.min(hi, x));
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    private double speedFromDistanceCm(double distanceCm) {
        final double d1 = 170.0;
        final double s1 = 1250.0;

        final double d2 = 304.8;
        final double s2 = 1450.0;

        double t = (distanceCm - d1) / (d2 - d1);
        t = clamp(t, 0.0, 1.0);

        return lerp(s1, s2, t);
    }


    @Override
    public void execute(){
        if (gamepad.getButton(GamepadKeys.Button.B)){
            state = MODE.ON;
        }
        if (gamepad.getButton(GamepadKeys.Button.A)){
            state = MODE.OFF;
        }

        List<Double> cameraData = cameraSubsystem.getCameraData();
        double range;

        if (cameraData == null){
            range = 0;
        }
        else {
            range = cameraData.get(1);
        }

        if (state == MODE.ON){
            double targetSpeed = speedFromDistanceCm(range);  // range assumed in cm
            launchSubsystem.setSpeed(-targetSpeed, telemetry);

            telemetry.addData("April Tag distance:", range);
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
