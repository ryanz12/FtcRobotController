package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BeltSubsyetm extends SubsystemBase {
    private CRServo BeltServoOne;
    private CRServo BeltServoTwo;
    public BeltSubsyetm(HardwareMap hMap){
        BeltServoOne = hMap.get(CRServo.class,"BeltServoOne");
        BeltServoTwo = hMap.get(CRServo.class,"BeltServoTwo");
    }
    public void MoveBelt(boolean dir){
        BeltServoOne.setPower(dir ? 1:-1);
        BeltServoTwo.setPower(dir ? -1:1);
    }


    public void StopIntake() {
        BeltServoOne.setPower(0);
        BeltServoTwo.setPower(0);
    }

}
