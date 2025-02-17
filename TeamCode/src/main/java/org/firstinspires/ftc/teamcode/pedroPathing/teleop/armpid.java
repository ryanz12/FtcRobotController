package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Config
public class armpid extends OpMode {
    private PIDController pid_controller;
    public static double p = 0, i = 0, d = 0;
    public static double f = 0;
    //W code
    // test
    public static int target_pos = 0;
    public final double ticks_in_degree = 537.7 / 360;

    private DcMotorEx arm_motor;

    @Override
    public void init(){
        pid_controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor = hardwareMap.get(DcMotorEx.class, "rotateMotor");
        arm_motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop(){
        pid_controller.setPID(p, i, d);
        int arm_pos = arm_motor.getCurrentPosition();
        double pid = pid_controller.calculate(arm_pos, target_pos);
        double ff = Math.sin(Math.toRadians(target_pos / ticks_in_degree)) * f;

        double power = pid + ff;

        arm_motor.setPower(power);

        telemetry.addData("pos ", arm_pos);
        telemetry.addData("target pos ", target_pos);
        telemetry.update();
    }
}
