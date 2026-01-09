package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BeltSubsystem extends SubsystemBase {
    private CRServo beltServoOne;
    private CRServo beltServoTwo;

    public enum Direction {
        UTR, DTR
    }

    public BeltSubsystem(HardwareMap hMap){
        beltServoOne = hMap.get(CRServo.class, "beltServoOne");
        beltServoTwo = hMap.get(CRServo.class, "beltServoTwo");
    }

    public void move_belt(Direction direction){
        beltServoOne.setPower(direction == Direction.UTR ? 1 : -1);
        beltServoTwo.setPower(direction == Direction.UTR ? -1 : 1);
    }

    public void stop() {
        beltServoOne.setPower(0);
        beltServoTwo.setPower(0);
    }
}
