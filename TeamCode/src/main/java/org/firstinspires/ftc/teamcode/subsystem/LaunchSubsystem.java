package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LaunchSubsystem extends SubsystemBase {
    private DcMotorEx launchMotor;

    private String launchMotorName = "launchMotor";

    public LaunchSubsystem(HardwareMap hwMap){
        launchMotor = hwMap.get(DcMotorEx.class, launchMotorName);
        launchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //https://www.reddit.com/r/FTC/comments/lfl82f/whats_the_difference_between_setvelocity_and/
        launchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void shoot(){
        launchMotor.setPower(0.6);
    }

    public void reverseShoot (){launchMotor.setPower(-1);}

    public void stop(){
        launchMotor.setPower(0);
    }
}