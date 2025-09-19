package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveSubsystem extends SubsystemBase {
    private Motor left_front;
    private Motor left_back;
    private Motor right_front;
    private Motor right_back;

    private MecanumDrive drive;

    private String left_front_name;
    private String left_back_name;
    private String right_front_name;
    private String right_back_name;

    public DriveSubsystem(HardwareMap hardwareMap){
        left_front = new Motor(hardwareMap, left_front_name, Motor.GoBILDA.RPM_312);
        left_back = new Motor(hardwareMap, left_back_name, Motor.GoBILDA.RPM_312);
        right_front = new Motor(hardwareMap, right_front_name, Motor.GoBILDA.RPM_312);
        right_back = new Motor(hardwareMap, right_back_name, Motor.GoBILDA.RPM_312);

        left_front.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        left_front.resetEncoder();
        left_back.resetEncoder();
        right_front.resetEncoder();
        right_back.resetEncoder();

        drive = new MecanumDrive(left_front, right_front, left_back, right_back);
    }

    public void drive(double strafe, double forward, double turn){
        drive.driveRobotCentric(strafe, forward, turn);
    }
}
