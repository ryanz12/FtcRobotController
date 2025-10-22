package org.firstinspires.ftc.teamcode.lab;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="MotorTestInstant",group = "test")
public class MotorTest extends LinearOpMode {

    private DcMotor intake;

    @Override
    public void runOpMode() {
        intake = hardwareMap.get(DcMotor.class, "leftFront");

        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.a) {
                intake.setPower(1);
            }
            else if(gamepad1.b){
                intake.setPower(-1);
            }
            telemetry.addData("Motor Status", "Running at full power");
            telemetry.update();
        }

        intake.setPower(0);
    }
}
