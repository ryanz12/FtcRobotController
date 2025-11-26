package org.firstinspires.ftc.teamcode.Auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class BlueAutoFar extends LinearOpMode {

    private class Launcher {
        private DcMotorEx launchMotor;

        // Initialize
        public Launcher(HardwareMap hwMap){
            launchMotor = hwMap.get(DcMotorEx.class, "launchMotor");
        }


        // Get some action going
        public Action shoot() {
            return new Action(){
                private boolean initalized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    if (!initalized){
                        launchMotor.setPower(0.8);
                        initalized = true;
                    }

                    // * Gets the velocity in ticks/second
                    double vel = launchMotor.getVelocity();
                    telemetryPacket.put("velocity", vel);

                    return vel < 10000.0;
                }
            };
        }

    }


    @Override
    public void runOpMode(){
        Launcher launcher = new Launcher(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(launcher.shoot());
    }
}
