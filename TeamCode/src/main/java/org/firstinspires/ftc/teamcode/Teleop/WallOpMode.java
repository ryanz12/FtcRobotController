package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoSetter", group = "Teleop")
public class WallOpMode extends LinearOpMode {

    private Servo wallServo;

    @Override
    public void runOpMode() {
        // Initialize the servo
        wallServo = hardwareMap.get(Servo.class, "wallServo");
        wallServo.setDirection(Servo.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.y){
                wallServo.setPosition(0);
            }
            // Press A for 0 degrees
            if (gamepad1.a) {
                wallServo.setPosition(1.0);
            }

            // Press B for 90 degrees
            if (gamepad1.b) {
                wallServo.setPosition(0.5);
            }


            // Display data on the Driver Station
            telemetry.addData("Servo Position", wallServo.getPosition());
            telemetry.addData("Target Angle", wallServo.getPosition() * 180.0);
            telemetry.update();
        }
    }
}