package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.common.Constants.Intake.INTAKE_POWER;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private CRServo leftIntake, rightIntake, centerIntake;



    public enum IntakeState{
        ACTIVE,
        INACTIVE
    }

    public IntakeState state;

    public void init(HardwareMap hardwareMap){
        leftIntake = hardwareMap.get(CRServo.class, "Left_intake");
        centerIntake = hardwareMap.get(CRServo.class, "Left_intake");
        rightIntake = hardwareMap.get(CRServo.class, "Left_intake");

        deActivate();

        state = IntakeState.INACTIVE;



    }


    public void activate(){
        leftIntake.setPower(INTAKE_POWER);
        centerIntake.setPower(INTAKE_POWER);
        rightIntake.setPower(INTAKE_POWER);

    }

    public void deActivate(){
        leftIntake.setPower(0);
        leftIntake.setPower(0);
        leftIntake.setPower(0);
    }

    public void toggle(){

    }
}
