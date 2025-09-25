package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class LaunchSubsystem extends SubsystemBase {
    private DcMotor launchMotor;

    public LaunchSubsystem(HardwareMap hwMap){
        launchMotor = hwMap.get(DcMotorEx.class, "launchMotor");
        launchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launchMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void shoot(){
        launchMotor.setPower(1.0);
    }
}
