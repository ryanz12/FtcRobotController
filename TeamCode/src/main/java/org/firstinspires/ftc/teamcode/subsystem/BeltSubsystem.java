package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BeltSubsystem extends SubsystemBase {
    private CRServo belt_servo_one;
    private CRServo belt_servo_two;

    private String beltServoOneName = "beltServoOne";
    private String beltServoTwoName = "beltServoTwo";
    
    public enum Direction {
        UTR, DTR
    }

    public BeltSubsystem(HardwareMap hMap){
        belt_servo_one = hMap.get(CRServo.class, beltServoOneName);
        belt_servo_two = hMap.get(CRServo.class, beltServoTwoName);
    }
    
    public void move_belt(Direction direction){
        belt_servo_one.setPower(direction == Direction.UTR ? 1 : -1);
        belt_servo_two.setPower(direction == Direction.UTR ? -1 : 1);
    }

    public void stop() {
        belt_servo_one.setPower(0);
        belt_servo_two.setPower(0);
    }
}
