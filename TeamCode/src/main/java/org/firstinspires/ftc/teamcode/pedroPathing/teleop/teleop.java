package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "arm test")
public class teleop extends LinearOpMode {
    private DcMotor arm_motor;

    @Override
    public void runOpMode(){
        arm_motor = hardwareMap.get(DcMotor.class, "specimenSlide");
        waitForStart();

        while(opModeIsActive()){
            arm_motor.setPower(-1);
        }
    }
}