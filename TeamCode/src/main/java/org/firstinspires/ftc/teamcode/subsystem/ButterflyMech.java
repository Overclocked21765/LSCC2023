package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.common.Constants.Drive.BFLY_MAX;
import static org.firstinspires.ftc.teamcode.common.Constants.Drive.BFLY_MIN;
import static org.firstinspires.ftc.teamcode.common.Constants.Drive.ODO_MAX;
import static org.firstinspires.ftc.teamcode.common.Constants.Drive.ODO_MIN;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ButterflyMech {
    private Servo frontL, frontR, backL, backR, odoLeft, odoRight, odoCenter;

    public void init(HardwareMap hardwareMap){
        frontL = hardwareMap.get(Servo.class, "Bfly_FrontL");
        frontR = hardwareMap.get(Servo.class, "Bfly_FrontR");
        backL = hardwareMap.get(Servo.class, "Bfly_BackL");
        backR = hardwareMap.get(Servo.class, "Bfly_BackR");
        odoLeft = hardwareMap.get(Servo.class, "odo_left");
        odoCenter = hardwareMap.get(Servo.class, "odo_center");
        odoRight = hardwareMap.get(Servo.class, "odo_right");

        frontL.scaleRange(BFLY_MIN, BFLY_MAX);
        frontR.scaleRange(BFLY_MIN, BFLY_MAX);
        backL.scaleRange(BFLY_MIN, BFLY_MAX);
        backR.scaleRange(BFLY_MIN, BFLY_MAX);

        odoLeft.scaleRange(ODO_MIN, ODO_MAX);
        odoCenter.scaleRange(ODO_MIN, ODO_MAX);
        odoRight.scaleRange(ODO_MIN, ODO_MAX);

        retractTank();

    }

    public void extendTank(){
        frontL.setPosition(1);
        frontR.setPosition(1);
        backL.setPosition(1);
        backR.setPosition(1);
    }

    public void retractTank(){
        frontL.setPosition(0);
        frontR.setPosition(0);
        backL.setPosition(0);
        backR.setPosition(0);
    }

    public void retractOdo(){
        odoLeft.setPosition(1);
        odoRight.setPosition(1);
        odoCenter.setPosition(1);
    }

}
