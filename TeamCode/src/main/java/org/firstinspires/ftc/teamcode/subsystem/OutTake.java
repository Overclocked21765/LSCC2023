package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.common.Constants.Outtake.OUTTAKE_ARM_MIN;
import static org.firstinspires.ftc.teamcode.common.Constants.Outtake.OUTTAKE_ARM_MAX;
import static org.firstinspires.ftc.teamcode.common.Constants.Outtake.OUTTAKE_ROTATE_MIN;
import static org.firstinspires.ftc.teamcode.common.Constants.Outtake.OUTTAKE_ROTATE_MAX;
import static org.firstinspires.ftc.teamcode.common.Constants.Outtake.OUTTAKE_POWER;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OutTake {
    private Servo leftArm, rightArm, leftRotate, rightRotate;
    private DcMotorEx rollers;

    public void init(HardwareMap hardwareMap){
        leftArm = hardwareMap.get(Servo.class, "Outtake_Left_Arm");
        rightArm = hardwareMap.get(Servo.class, "Outtake_Right_Arm");
        leftRotate = hardwareMap.get(Servo.class, "left_rotate");
        rightRotate = hardwareMap.get(Servo.class, "right_rotate");

        rollers = hardwareMap.get(DcMotorEx.class, "Outtake_motor");

        leftArm.scaleRange(OUTTAKE_ARM_MIN, OUTTAKE_ARM_MAX);
        rightArm.scaleRange(OUTTAKE_ARM_MIN, OUTTAKE_ARM_MAX);

        leftRotate.scaleRange(OUTTAKE_ROTATE_MIN, OUTTAKE_ROTATE_MAX);
        rightRotate.scaleRange(OUTTAKE_ROTATE_MIN, OUTTAKE_ROTATE_MAX);

        rollers.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rollers.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void receiveAndHold(){
        rollers.setPower(OUTTAKE_POWER);
    }

    public void releaseNormal(){
        rollers.setPower(-OUTTAKE_POWER);
    }

    public void releaseLeft(){
        rollers.setPower(-OUTTAKE_POWER);
        leftArm.setPosition(1);
    }

    public void releaseRight(){
        rollers.setPower(-OUTTAKE_POWER);
        rightArm.setPosition(1);
    }

    public void zeroIntake(){
        leftArm.setPosition(0);
        rightArm.setPosition(0);
        rollers.setPower(0);
        moveToZero();
    }

    public void moveToDeposit(){
        leftRotate.setPosition(1);
        rightRotate.setPosition(1);
    }

    public void moveToZero(){
        leftRotate.setPosition(0);
        rightRotate.setPosition(0);
    }


}
