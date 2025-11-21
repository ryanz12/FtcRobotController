package org.firstinspires.ftc.teamcode.lab;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp(name = "MotorMaxVelocity")
public class MotorMaxVelocity extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "launchMotor");

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Open loop
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        motor.setPower(1.0);   // Full power

        double maxV = 0;

        while (opModeIsActive()) {

            double v = motor.getVelocity(); // ticks per second

            if (v > maxV) {
                maxV = v;
            }

            telemetry.addData("Current Velocity", v);
            telemetry.addData("Max Velocity Recorded", maxV);
            telemetry.update();
        }

        motor.setPower(0);
    }
}
