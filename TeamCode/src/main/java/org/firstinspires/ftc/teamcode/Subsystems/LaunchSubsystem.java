package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class LaunchSubsystem extends SubsystemBase {
    private DcMotorEx launchMotor;
    private double currentVelocity;
    public double target = 0;
    private double kP = 0.003;
    private double kF = 0.0004167;
    private double error;
    private double output;
    private Telemetry telemetry;
    public static double k = 2.5;

    public LaunchSubsystem(HardwareMap hMap, Telemetry tel){
        launchMotor = hMap.get(DcMotorEx.class, "launchMotor");
        launchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.telemetry = tel;
    }

    public void setSpeed(double vel, Telemetry tel){
        target = vel;
    }

    @Override
    public void periodic(){
        currentVelocity = launchMotor.getVelocity();

        error = target-currentVelocity;

        output = kF * target + kP * error;

        if (Math.abs(Math.abs(currentVelocity) - Math.abs(target)) < 30 && target != 0){
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

    public void stop(){
        target = 0;
        launchMotor.setPower(0);
    }
}
