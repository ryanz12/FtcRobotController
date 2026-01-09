package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveSubsystem extends SubsystemBase {
    private Motor leftFront;
    private Motor leftBack;
    private Motor rightFront;
    private Motor rightBack;
    private MecanumDrive drivebase;
    private final double STRAFE_MULTIPLIER = 0.75;
    public DriveSubsystem(HardwareMap hMap){
        leftFront = new Motor(hMap, "leftFront", Motor.GoBILDA.RPM_312);
        leftBack = new Motor(hMap, "leftBack", Motor.GoBILDA.RPM_312);
        rightFront = new Motor(hMap, "rightFront", Motor.GoBILDA.RPM_312);
        rightBack = new Motor(hMap, "rightBack", Motor.GoBILDA.RPM_312);

        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        leftFront.resetEncoder();
        leftBack.resetEncoder();
        rightFront.resetEncoder();
        rightBack.resetEncoder();

        drivebase = new MecanumDrive(leftFront, rightFront, leftBack, rightBack);
    }

    public void drive(double strafe, double forward, double turn){
        drivebase.driveRobotCentric(strafe*STRAFE_MULTIPLIER, forward, turn);
    }
}
