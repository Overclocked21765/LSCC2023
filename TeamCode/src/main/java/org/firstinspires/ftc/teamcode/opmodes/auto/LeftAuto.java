package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.subsystem.ButterflyMech;
import org.firstinspires.ftc.teamcode.subsystem.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Lift;
import org.firstinspires.ftc.teamcode.subsystem.OutTake;
import org.firstinspires.ftc.teamcode.subsystem.TSEClimb;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment;

@Autonomous
public class LeftAuto extends OpMode {
    public enum States{
        START,
        WAITING_TO_DEPOSIT,
        PREP_FOR_PARK,
        EMPTY_TRANSPORT,
        PREP,
        FULL_TRANSPORT,
        GOING_FOR_PARK,
        DONE
    }

    SampleMecanumDrive drive;
    DriveTrain butterfly = new DriveTrain();
    Intake intake = new Intake();
    Lift lift = new Lift();
    OutTake outtake = new OutTake();
    TSEClimb tseClimb = new TSEClimb();

    ElapsedTime timer = new ElapsedTime();

    TrajectorySequence tse;
    TrajectorySequence retrieve;
    TrajectorySequence score;
    TrajectorySequence readyForPark;
    TrajectorySequence nextTrajectory;

    double zone;

    boolean readyforpark;

    States state;



    @Override
    public void init(){
        drive = new SampleMecanumDrive(hardwareMap);
        intake.init(hardwareMap);
        lift.init(hardwareMap);
        outtake.init(hardwareMap);
        tseClimb.init(hardwareMap);

        tseClimb.grab();
        intake.deActivate();
        outtake.zeroIntake();
        outtake.receiveAndHold();

        tse = drive.trajectorySequenceBuilder(new Pose2d(-15, -63, Math.toRadians(-90)))
                .setReversed(true)
                        .splineTo(new Vector2d(-15, -45), Math.toRadians(90))
                                .splineTo(new Vector2d(-17, -42), Math.toRadians(110))
                                        .build();

        retrieve = drive.trajectorySequenceBuilder(tse.end())
                .setReversed(false)
                .splineTo(new Vector2d(1.00, -40.33), Math.toRadians(0.00))
                .splineTo(new Vector2d(11.33, -30.00), Math.toRadians(53.13))
                .splineTo(new Vector2d(21.83, -11.17), Math.toRadians(45.00))
                .splineTo(new Vector2d(35.83, 3.33), Math.toRadians(71.57))
                .splineTo(new Vector2d(53.33, 30.50), Math.toRadians(90.00))
                .splineTo(new Vector2d(60.17, 48.00), Math.toRadians(68.67))
                .splineTo(new Vector2d(62.17, 53.00), Math.toRadians(68.20))
                .build();

        score = drive.trajectorySequenceBuilder(retrieve.end())
                .setReversed(true)
                .splineTo(new Vector2d(1.00, -40.33), Math.toRadians(0.00))
                .splineTo(new Vector2d(11.33, -30.00), Math.toRadians(53.13))
                .splineTo(new Vector2d(21.83, -11.17), Math.toRadians(45.00))
                .splineTo(new Vector2d(35.83, 3.33), Math.toRadians(71.57))
                .splineTo(new Vector2d(53.33, 30.50), Math.toRadians(90.00))
                .splineTo(new Vector2d(60.17, 48.00), Math.toRadians(68.67))
                .splineTo(new Vector2d(62.17, 53.00), Math.toRadians(68.20))
                .build();

        readyForPark = drive.trajectorySequenceBuilder(score.end())
                .splineTo(new Vector2d(-0.83, -41.5), Math.toRadians(0.00))
                .splineTo(new Vector2d(13.17, -41.5), Math.toRadians(0))
                .build();








        drive.setPoseEstimate(tse.start());


        readyforpark = false;

    }

    @Override
    public void init_loop(){
        zone = 1;
    }

    @Override
    public void start(){
        drive.followTrajectorySequenceAsync(tse);
        state = States.START;
        if (zone == 1){
            tseClimb.zone1();
        } else if (zone == 2){
            tseClimb.zone2();
        } else {
            tseClimb.zone3();
        }
        lift.depositPos();
    }

    @Override
    public void loop(){
        switch (state){
            case START:
                if (!drive.isBusy()){
                    tseClimb.release();
                    outtake.releaseNormal();
                    timer.reset();
                    state = States.WAITING_TO_DEPOSIT;
                    nextTrajectory = retrieve;
                }
                break;
            case WAITING_TO_DEPOSIT:
                if (timer.time() > 0.2){
                    tseClimb.reset();
                    outtake.zeroIntake();
                    lift.groundPos();
                    outtake.receiveAndHold();
                    timer.reset();
                    if (readyforpark){
                        state = States.PREP_FOR_PARK;
                    } else {
                        state = States.EMPTY_TRANSPORT;
                    }
                    drive.followTrajectorySequenceAsync(nextTrajectory);
                }
                break;
            case EMPTY_TRANSPORT:
                if (timer.time() > 4){
                    intake.activate();
                    state = States.PREP;

                }
                break;
            case PREP:
                if (!drive.isBusy()){
                    outtake.moveToDeposit();
                    lift.depositPos();
                    intake.deActivate();
                    state = States.FULL_TRANSPORT;
                    drive.followTrajectorySequenceAsync(score);

                }
                break;
            case FULL_TRANSPORT:
                if (!drive.isBusy()){
                    outtake.releaseNormal();
                    timer.reset();
                    state = States.WAITING_TO_DEPOSIT;
                    nextTrajectory = readyForPark;
                    readyforpark = true;

                }
                break;
            case PREP_FOR_PARK:
                if (!drive.isBusy()){
                    butterfly.init(hardwareMap);
                    butterfly.extendTank();
                    butterfly.retractOdo();
                    drive = null;
                    butterfly.drive(0, -1, 0, -1);
                    state = States.GOING_FOR_PARK;
                    timer.reset();
                }
                break;
            case GOING_FOR_PARK:
                if (timer.time() > 3){
                    butterfly.drive(0, 0, 0, 0);
                    intake.deActivate();
                    outtake.zeroIntake();
                    lift.groundPos();
                    state = States.DONE;
                }
        }
        lift.update();
        drive.update();

        telemetry.addData("STATE: ", state);

    }
}
