package org.firstinspires.ftc.teamcode.lab;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@TeleOp(name = "IMU")
public class IMUExample extends LinearOpMode {

    BNO055IMU imu;

    @Override
    public void runOpMode() {

        // Initialize the IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Set up IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);

        // Wait for IMU to calibrate
        telemetry.addLine("Calibrating IMU...");
        telemetry.update();
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addLine("IMU Calibrated. Waiting for start...");
        telemetry.update();

        waitForStart();

        // Start acceleration integration (this enables position updates)
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        while (opModeIsActive()) {
            // Get orientation
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double heading = angles.firstAngle;
            // Get current position and velocity
            Position curPos = imu.getPosition();
            Velocity vel = imu.getVelocity();

            //Telemetry
            telemetry.addData("Heading (deg)", heading);
            telemetry.addData("Position (m)", "x=%.2f, y=%.2f, z=%.2f", curPos.x, curPos.y, curPos.z);
            telemetry.addData("Velocity (m/s)", "x=%.2f, y=%.2f, z=%.2f", vel.xVeloc, vel.yVeloc, vel.zVeloc);
            telemetry.update();

            sleep(100);
        }
    }
}
