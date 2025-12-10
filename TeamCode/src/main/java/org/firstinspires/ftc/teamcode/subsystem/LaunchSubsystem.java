package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LaunchSubsystem extends SubsystemBase {
    private DcMotorEx launchMotor;

    private String launchMotorName = "launchMotor";

    private Telemetry telemetry;

    private double current_velocity;
    private double target_velocity;

    private double kP = 0.003;
    private double kI = 0;
    private double kD = 0.01;
    private double kF = 0.0004167;
    private double voltage_constant = 0.95;
    private double voltage_reference = 13.0;
    private double voltage;
    private double error, error_sum, prev_error, delta_error, delta_time;
    private double output;

    public LaunchSubsystem(HardwareMap hwMap, Telemetry telemetry){
        this.telemetry = telemetry;

        launchMotor = hwMap.get(DcMotorEx.class, launchMotorName);
        launchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //https://www.reddit.com/r/FTC/comments/lfl82f/whats_the_difference_between_setvelocity_and/
        launchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void shoot(double power){
        launchMotor.setPower(power);
//        launchMotor.setVelocity(power * maxTickPerSec);
    }

    public void set_pid(double target_velocity){
        current_velocity = launchMotor.getVelocity();


        error = target_velocity-current_velocity;
//        error_sum += error;

//        delta_error = (error-prev_error)/delta_time;
//        prev_error = error;

        double ff = kF * target_velocity;

        output = ff + kP * error;

        if (Math.abs(Math.abs(current_velocity) - Math.abs(target_velocity)) < 30 && target_velocity != 0){
            telemetry.addLine("READY!!");
            telemetry.addLine("READY!!");
            telemetry.addLine("READY!!");
        }
        else {
            telemetry.addLine("");
        }

        telemetry.update();

        launchMotor.setPower(-output);
    }

    public void reverseShoot (){
        launchMotor.setPower(-1);
//        launchMotor.setVelocity(-maxTickPerSec);
    }

    public void stop(){

        launchMotor.setPower(0);
//        launchMotor.setVelocity(0);
    }
}