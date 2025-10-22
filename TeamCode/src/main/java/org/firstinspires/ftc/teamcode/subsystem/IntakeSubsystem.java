package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends SubsystemBase {

    private DcMotor intake_motor;

    private String intake_motor_name = "intakeMotor";

    public IntakeSubsystem(HardwareMap hMap){
        intake_motor = hMap.get(DcMotorEx.class, intake_motor_name);
        intake_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void intake(){
        intake_motor.setPower(1);
    }

    public void stop(){
        intake_motor.setPower(0);
    }

}
