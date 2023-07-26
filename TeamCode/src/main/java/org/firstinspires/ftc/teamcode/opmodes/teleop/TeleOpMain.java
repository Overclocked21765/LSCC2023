package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.subsystem.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Lift;
import org.firstinspires.ftc.teamcode.subsystem.OutTake;
import org.firstinspires.ftc.teamcode.subsystem.TSEClimb;

@TeleOp
public class TeleOpMain extends OpMode {
    private DriveTrain driveTrain = new DriveTrain();
    private Intake intake = new Intake();
    private OutTake outTake = new OutTake();
    private TSEClimb tseClimb = new TSEClimb();
    private Lift lift = new Lift();

    private ElapsedTime timer = new ElapsedTime();

    boolean tankButtonPressed;
    boolean yAlreadyPressed;
    boolean xAlreadyPressed;
    boolean bAlreadyPressed;
    boolean imuButtonAlreadyPressed;
    boolean commanded;

    double depositDouble;

    public enum States{
        INTAKE_GOING_TO_DEPOSIT,
        DEPOSITED_GOING_TO_INTAKE,
        WAITING_ON_OUTTAKE
    }

    States state;

    @Override
    public void init(){
        driveTrain.initFull(hardwareMap);
        intake.init(hardwareMap);
        outTake.init(hardwareMap);
        tseClimb.init(hardwareMap);
        lift.init(hardwareMap);

        tankButtonPressed = false;
        bAlreadyPressed = false;
        commanded = false;
        xAlreadyPressed = false;
        yAlreadyPressed = false;
        imuButtonAlreadyPressed = false;

        state = States.INTAKE_GOING_TO_DEPOSIT;
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

        if (gamepad2.y && imuButtonAlreadyPressed){
            driveTrain.resetIMU();
        }
        imuButtonAlreadyPressed = gamepad2.y;


        if (gamepad1.y && !yAlreadyPressed){
            commanded = true;
            depositDouble = 0;
        }
        yAlreadyPressed = gamepad1.y;

        if (gamepad1.b && !bAlreadyPressed){
            commanded = true;
            depositDouble = 1;
        }
        bAlreadyPressed = gamepad1.b;

        if (gamepad1.x && !xAlreadyPressed){
            commanded = true;
            depositDouble = 2;
        }
        xAlreadyPressed = gamepad1.x;

        switch (state){
            case INTAKE_GOING_TO_DEPOSIT:
                if (commanded){
                    commanded = false;
                    intake.deActivate();
                    outTake.moveToDeposit();
                    lift.depositPos();
                    state = States.DEPOSITED_GOING_TO_INTAKE;
                }
                break;
            case DEPOSITED_GOING_TO_INTAKE:
                if (commanded){
                    commanded = false;
                    if (depositDouble == 1){
                        outTake.releaseRight();
                    } else if (depositDouble == 2){
                        outTake.releaseLeft();
                    } else {
                        outTake.releaseNormal();
                    }
                    outTake.zeroIntake();
                    lift.groundPos();
                }
                break;
        }
        lift.update();

    }
}
