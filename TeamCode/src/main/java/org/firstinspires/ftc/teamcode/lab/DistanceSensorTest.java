package org.firstinspires.ftc.teamcode.lab;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="DistanceSensorTest", group="test")
public class DistanceSensorTest extends LinearOpMode {

    private DistanceSensor distanceSensor;

    @Override
    public void runOpMode() {
        // Get the distance sensor from hardware map
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        waitForStart();

        while (opModeIsActive()) {
            // Read the distance in centimeters
            double distanceCm = distanceSensor.getDistance(DistanceUnit.CM);

            // Print it to telemetry
            telemetry.addData("Distance (cm)", distanceCm);
            telemetry.addData("Distance (inches)", distanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.update();

            // Optional small delay
            sleep(100);
        }
    }
}
