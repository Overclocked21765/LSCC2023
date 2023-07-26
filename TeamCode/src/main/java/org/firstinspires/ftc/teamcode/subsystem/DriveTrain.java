package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.Algorithms;

public class DriveTrain extends ButterflyMech{
    public enum DTState{
        EXTEND,
        RETRACT
    }

    private DcMotorEx fLeft, fRight, bLeft, bRight;
    private IMU imu;
    public DTState state;


    public void initFull(HardwareMap hardwareMap){
        init(hardwareMap);
        fLeft = hardwareMap.get(DcMotorEx.class, "Motor_FL");
        fRight = hardwareMap.get(DcMotorEx.class, "Motor_FR");
        bLeft = hardwareMap.get(DcMotorEx.class, "Motor_BL");
        bRight = hardwareMap.get(DcMotorEx.class, "Motor_BR");

        fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        imu = hardwareMap.get(IMU.class, "imu");

        state = DTState.RETRACT;
    }

    public void drive(double leftX, double leftY, double rightX, double rightY){
        switch (state){
            case EXTEND:
                fLeft.setPower(-leftY);
                fRight.setPower(-leftY);
                bLeft.setPower(-rightY);
                bRight.setPower(-rightY);
                break;
            case RETRACT:
                double forward = -leftY;
                double strafe = leftX;
                double rotation = rightX;
                YawPitchRollAngles yawPitchRollAngles = imu.getRobotYawPitchRollAngles();
                double heading = yawPitchRollAngles.getYaw(AngleUnit.DEGREES);
                double[] powers = Algorithms.returnMecanumValues(rotation, strafe, forward, heading, Constants.Drive.DRIVE_POWER_MODIFIER);
                fLeft.setPower(powers[0]);
                fRight.setPower(powers[1]);
                bLeft.setPower(powers[2]);
                bRight.setPower(powers[3]);
                break;

        }
    }

    public void toggleButterfly(){
        switch (state){
            case EXTEND:
                retractTank();
                state = DTState.RETRACT;
                break;
            case RETRACT:
                extendTank();
                state = DTState.EXTEND;
                break;
        }
    }





}
