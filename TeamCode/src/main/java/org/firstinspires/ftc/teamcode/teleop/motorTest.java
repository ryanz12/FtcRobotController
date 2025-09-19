package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="MotorTestInstant")
public class motorTest extends LinearOpMode {

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
