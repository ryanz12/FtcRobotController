package org.firstinspires.ftc.teamcode.lab;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp
public class MotorPID extends LinearOpMode {

    // Measure the ticks and time
    // Basically find the something that relates to the voltage
    // Make a pid controller w/ it


    // GOAL: make sure the ticks/second is the same or within a small
    // range

    // set a target tick speed
    // try to consistently reach the tick speed
    // check for discrepancies

    DcMotorEx launchMotor;

    ElapsedTime timer;

    VoltageSensor voltage_sensor;

    TelemetryPacket packet = new TelemetryPacket();

    // Velocity is measured in ticks/second
    public static double current_velocity;
    public static double target_velocity = -1300;
    final double maxV = -2400;

    double output;

    public static double kP = 0.002;
    public static double kI = 0;
    public static double kD = 0.01;
    public static double kF = 0.0004167;
    public static double voltage_constant = 0.95;

    double error;
    double error_sum;
    double prev_error;
    double delta_error;
    double delta_time;
    double voltage;

    final double voltage_reference = 13.0;

    @Override
    public void runOpMode(){
        launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
        launchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        launchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        timer = new ElapsedTime();

        voltage_sensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        waitForStart();
        timer.reset();

        double last_time = timer.seconds();

        while (opModeIsActive()){
            current_velocity = launchMotor.getVelocity();

            error = target_velocity - current_velocity;
            error_sum += error;

            delta_time = (timer.seconds() - last_time) > 0 ? timer.seconds() - last_time : 1e-3;

            delta_error = (error-prev_error)/delta_time;

            voltage = voltage_sensor.getVoltage();

            // turns out we probably won't need this but it's here if we do
            double ffv = kF * target_velocity * (voltage_reference/voltage)*voltage_constant;
            double ff = kF * target_velocity;

            output = kF * target_velocity + kP * error + kI * error_sum + kD * delta_error;
            launchMotor.setPower(-output);

            prev_error = error;

            telemetry.addData("target velocity", target_velocity);
            telemetry.addData("actual velocity", current_velocity);
            packet.put("voltage", voltage);
            packet.put("FF adjusted for voltage", ffv);
            packet.put("FF without voltage calc", ff);
            packet.put("motor output", output);
            packet.put("target", target_velocity);
            packet.put("real", current_velocity);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
}
