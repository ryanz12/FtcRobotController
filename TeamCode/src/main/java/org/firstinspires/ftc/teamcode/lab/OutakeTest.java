package org.firstinspires.ftc.teamcode.lab;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class OutakeTest extends LinearOpMode {

    private DcMotor leftShooter,rightShooter,intake;

    @Override
    public void runOpMode() {
        leftShooter = hardwareMap.get(DcMotor.class, "leftFront");
        rightShooter = hardwareMap.get(DcMotor.class,"leftBack");
        intake = hardwareMap.get(DcMotor.class,"rightFront");

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.a){
                leftShooter.setPower(1);
                rightShooter.setPower(-1);
            }
            else if(gamepad1.b){
                leftShooter.setPower(-1);
                rightShooter.setPower(1);
            }
            telemetry.addData("Motor Status", "Running at full power");
            telemetry.update();
        }

        leftShooter.setPower(0);
        rightShooter.setPower(0);
    }
}
