package org.firstinspires.ftc.teamcode.pedroPathing.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RotateSlideSubsystem extends SubsystemBase {
    private DcMotor rotate_motor;

    public RotateSlideSubsystem(HardwareMap hardwareMap){
        rotate_motor = hardwareMap.get(DcMotorEx.class, "rotateMotor");
        rotate_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotate_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        rotate_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotate_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void rotate_slides(int target_position, double speed){
        rotate_motor.setTargetPosition(target_position);
        rotate_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotate_motor.setPower(speed);
    }
    public boolean is_slide_busy(){
        // Returns true if slide is busy, else returns false
        return rotate_motor.isBusy();
    }
    public void stop_rotating(){
        rotate_motor.setPower(0);
        rotate_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
