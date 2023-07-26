package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.common.Constants.Lift.DEPOSIT_POSITION;
import static org.firstinspires.ftc.teamcode.common.Constants.Lift.GROUND_POSITION;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.PIDController;

public class Lift {
    private DcMotorEx leftMotor, rightMotor;

    private PIDController controller;

    public double slidePos;
    public double target;

    public static double kF = 0.24;
    public static double kP = 0.25;
    public static double kI = 0;
    public static double kD = 0.8;

    public void init(HardwareMap hardwareMap){
        leftMotor = hardwareMap.get(DcMotorEx.class, "Lift_Left");
        rightMotor = hardwareMap.get(DcMotorEx.class, "Lift_Right");

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        controller = new PIDController(kP, kI, kD, 0);

        this.target = GROUND_POSITION;

        update();



    }

    public void update(){
        slidePos = leftMotor.getCurrentPosition();
        double power = controller.calculate(slidePos, this.target) - kF;
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    public void setTarget(double target){
        this.target = target;
    }

    public void depositPos(){
        this.target = DEPOSIT_POSITION;
    }

    public void groundPos(){
        this.target = GROUND_POSITION;
    }
}
