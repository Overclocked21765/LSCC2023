package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.DriveTrain;

@TeleOp
public class TeleOpMain extends OpMode {
    private DriveTrain driveTrain = new DriveTrain();

    boolean tankButtonPressed;

    @Override
    public void init(){
        driveTrain.initFull(hardwareMap);

        tankButtonPressed = false;
    }

    @Override
    public void start(){
        driveTrain.retractOdo();
    }

    @Override
    public void loop(){
        driveTrain.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y);

        if (gamepad1.a && !tankButtonPressed){
            driveTrain.toggleButterfly();
        }



        tankButtonPressed = gamepad1.a;

    }
}
