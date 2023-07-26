package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.common.Constants.TSECLimb.CLIMB;
import static org.firstinspires.ftc.teamcode.common.Constants.TSECLimb.ZONE_ONE;
import static org.firstinspires.ftc.teamcode.common.Constants.TSECLimb.ZONE_TWO;
import static org.firstinspires.ftc.teamcode.common.Constants.TSECLimb.ZONE_THREE;
import static org.firstinspires.ftc.teamcode.common.Constants.TSECLimb.GRIP_MIN;
import static org.firstinspires.ftc.teamcode.common.Constants.TSECLimb.GRIP_MAX;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TSEClimb {
    private DcMotorEx motor;
    private Servo grip;

    public void init(HardwareMap hardwareMap){
        motor = hardwareMap.get(DcMotorEx.class, "TSE_Motor");
        grip = hardwareMap.get(Servo.class, "TSE_Servo");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setPower(1);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        grip.scaleRange(GRIP_MIN, GRIP_MAX);
    }

    public void grab(){
        grip.setPosition(1);
    }

    public void climb(){
        motor.setTargetPosition(CLIMB);

    }

    public void release(){
        grip.setPosition(0);
    }

    public void reset(){
        motor.setTargetPosition(0);
        grip.setPosition(0);
    }

    public void zone1(){
        motor.setTargetPosition(ZONE_ONE);
    }

    public void zone2(){
        motor.setTargetPosition(ZONE_TWO);
    }

    public void zone3(){
        motor.setTargetPosition(ZONE_THREE);
    }


}
