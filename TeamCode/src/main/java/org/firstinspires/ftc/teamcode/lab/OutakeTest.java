package org.firstinspires.ftc.teamcode.lab;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="OutakeTest", group="Shooter")
public class OutakeTest extends LinearOpMode {

    private DcMotorEx shooterMotor;

    static final double g = 9.81; // gravity m/s^2
    static final double launchAngleDeg = 43.5; // fixed angle
    static final double launchAngleRad = Math.toRadians(launchAngleDeg);

    @Override
    public void runOpMode() {

        // Initialize motor
        shooterMotor = hardwareMap.get(DcMotorEx.class, "outakeMotor");

        // Ensure motor can use velocity control
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        waitForStart();

        if (opModeIsActive()) {

            // Example target input
            double xTarget = 2.0; // meters
            double yTarget = 2.0; // meters
            double flywheelRadius = 0.05; // meters
            double gearRatio = 1.0; // output:input

            // Compute required launch speed for fixed angle
            double launchSpeed = requiredSpeed(xTarget, yTarget, launchAngleRad);

            if (Double.isNaN(launchSpeed)) {
                telemetry.addData("Error", "Target unreachable at fixed angle!");
            } else {
                // Convert launch speed to flywheel RPM
                double flywheelRPM = (launchSpeed / (2 * Math.PI * flywheelRadius)) * 60 / gearRatio;

                telemetry.addData("Launch Speed (m/s)", launchSpeed);
                telemetry.addData("Launch Angle (deg)", launchAngleDeg);
                telemetry.addData("Flywheel RPM", flywheelRPM);
                telemetry.update();

                // Spin motor to target velocity
                shooterMotor.setVelocity(flywheelRPM * 28 / 60); // 28 ticks per rev example

                sleep(3000); // spin for 3 seconds
                shooterMotor.setPower(0);
            }
        }
    }

    // Required launch speed formula from projectile motion
    private double requiredSpeed(double x, double y, double angle) {
        double cos = Math.cos(angle);
        double denominator = 2 * cos * cos * (x * Math.tan(angle) - y);
        if (denominator <= 0) return Double.NaN; // impossible
        return Math.sqrt((g * x * x) / denominator);
    }
}
