package org.firstinspires.ftc.teamcode.Auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous
public class BlueAutoFar extends LinearOpMode {

    private class Launcher {
        private DcMotorEx launchMotor;

        // Initialize
        public Launcher(HardwareMap hwMap){
            launchMotor = hwMap.get(DcMotorEx.class, "launchMotor");
            launchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }


        // Get some action going
        public Action shoot() {
            return new Action(){
                private boolean initalized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    if (!initalized){
                        launchMotor.setPower(1);
                        initalized = true;
                    }

                    // * Gets the velocity in ticks/second
                    double vel = launchMotor.getVelocity();
                    telemetryPacket.put("velocity", vel);

                    if (vel < -1450.0){
                        launchMotor.setPower(0);
                        return false;
                    }

                    return true;
                }
            };
        }
    }

    private class Ramp {
        private CRServo rampServoOne, rampServoTwo;
        private ElapsedTime timer;

        public Ramp(HardwareMap hwMap){
            rampServoOne = hwMap.get(CRServo.class, "beltServoOne");
            rampServoTwo = hwMap.get(CRServo.class, "beltServoTwo");

            timer = new ElapsedTime();
        }

        public Action rampUp(){
            return new Action(){

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    rampServoOne.setPower(1);
                    rampServoTwo.setPower(-1);

                    telemetryPacket.put("running true", true);

                    if (timer.seconds() > 3){
                        rampServoOne.setPower(0);
                        rampServoTwo.setPower(0);
                        return false;
                    }

                    return true;
                }
            };
        }
    }


    @Override
    public void runOpMode(){
        Launcher launcher = new Launcher(hardwareMap);
        Ramp ramp = new Ramp(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(
                launcher.shoot(),
                ramp.rampUp()

        ));
    }
}
