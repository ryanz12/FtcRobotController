package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OuttakeSubsystem extends SubsystemBase {
    private CRServo outtakeServo;
    public OuttakeSubsystem(HardwareMap hMap){
        outtakeServo = hMap.get(CRServo.class, "outtakeServo");
    }

    public void intake(){
        outtakeServo.setPower(-1);
    }

    public void outtake(){
        outtakeServo.setPower(0.75);
    }

    public void stop(){
        outtakeServo.setPower(0);
    }
}
