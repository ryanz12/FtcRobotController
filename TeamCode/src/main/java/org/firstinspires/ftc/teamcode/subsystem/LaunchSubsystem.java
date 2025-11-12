package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LaunchSubsystem extends SubsystemBase {
    private DcMotor launchMotor;

    private String launchMotorName = "launchMotor";

    public LaunchSubsystem(HardwareMap hwMap){
        launchMotor = hwMap.get(DcMotorEx.class, launchMotorName);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //https://www.reddit.com/r/FTC/comments/lfl82f/whats_the_difference_between_setvelocity_and/
        launchMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void shoot(){
        //test
        launchMotor.setPower(1);
    }

    public void stop(){
        launchMotor.setPower(0);
    }
}