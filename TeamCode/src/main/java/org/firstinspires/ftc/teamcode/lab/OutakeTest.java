package org.firstinspires.ftc.teamcode.lab;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name="OutakeTest", group="test")
public class OutakeTest extends LinearOpMode {

    private DcMotorEx shooterMotor;
    private VoltageSensor voltageSensor;

    static final double g = 9.81; // gravity m/s^2
    static final double launchAngleDeg = 43.5; // fixed angle
    static final double launchAngleRad = Math.toRadians(launchAngleDeg);

    // Nominal battery voltage the PIDF was tuned for (typically 12V)
    static final double NOMINAL_VOLTAGE = 12.0;

    @Override
    public void runOpMode() {

        shooterMotor = hardwareMap.get(DcMotorEx.class, "outakeMotor");
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Get battery voltage sensor
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        waitForStart();

        if (opModeIsActive()) {

            double xTarget = 2.0; // meters
            double yTarget = 1.5; // meters
            double flywheelRadius = 0.05; // meters
            double gearRatio = 1.0; // output:input

            double launchSpeed = requiredSpeed(xTarget, yTarget, launchAngleRad);

            if (Double.isNaN(launchSpeed)) {
                telemetry.addData("Error", "Target unreachable at fixed angle!");
            } else {

                double flywheelRPM = (launchSpeed / (2 * Math.PI * flywheelRadius)) * 60 / gearRatio;

                telemetry.addData("Launch Speed (m/s)", launchSpeed);
                telemetry.addData("Launch Angle (deg)", launchAngleDeg);
                telemetry.addData("Flywheel RPM", flywheelRPM);

                // --- Voltage Compensation for PIDF ---
                double currentVoltage = voltageSensor.getVoltage();
                double compensatedF = 1.0 * (NOMINAL_VOLTAGE / currentVoltage); // scale F term
                double compensatedP = 0.1;
                double compensatedI = 0.1;
                double compensatedD = 0.5;

                shooterMotor.setVelocityPIDFCoefficients(
                        compensatedP,
                        compensatedI,
                        compensatedD,
                        compensatedF
                );

                telemetry.addData("Voltage", currentVoltage);
                telemetry.addData("F Coefficient (Scaled)", compensatedF);
                telemetry.update();

                // Convert RPM → ticks/sec (for 1120-tick motor)
                double targetVelocity = flywheelRPM * 1120 / 60.0;
                shooterMotor.setVelocity(targetVelocity);

                sleep(3000); // spin for 3 seconds
                shooterMotor.setPower(0);
            }
        }
    }

    private double requiredSpeed(double x, double y, double angle) {
        double cos = Math.cos(angle);
        double denominator = 2 * cos * cos * (x * Math.tan(angle) - y);
        if (denominator <= 0) return Double.NaN;
        return Math.sqrt((g * x * x) / denominator);
    }
}
