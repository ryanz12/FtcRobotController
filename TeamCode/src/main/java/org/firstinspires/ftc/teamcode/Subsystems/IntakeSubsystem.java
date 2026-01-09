package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends SubsystemBase {
    private DcMotorEx intakeMotor;
    public IntakeSubsystem(HardwareMap hMap){
        intakeMotor = hMap.get(DcMotorEx.class, "intakeMotor");
    }

    public void intake(){
        intakeMotor.setPower(1);
    }

    public void outtake(){
        intakeMotor.setPower(-1);
    }

    public void stop(){
        intakeMotor.setPower(0);
    }
}
