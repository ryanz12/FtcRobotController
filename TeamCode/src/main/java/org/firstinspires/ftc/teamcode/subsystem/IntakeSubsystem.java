package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends SubsystemBase {

    private final CRServo intake_servo;
    private final String intake_servo_name = "intakeServo";

    public IntakeSubsystem(HardwareMap hMap) {
        intake_servo = hMap.get(CRServo.class, intake_servo_name);
    }

    public void intake() {
        intake_servo.setPower(-1);
    }

    public void outtake() {
        intake_servo.setPower(1);
    }

    public void stop() {
        intake_servo.setPower(0);
    }
}
