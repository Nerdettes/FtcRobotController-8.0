package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "PinpointLeftPath", group = "")
public class PinpointLeftPath extends LinearOpMode {
    private static final int NUMLOOPS = 3 ;
    //test1

    private PinpointDrive drive;
    //private actuatorUtils utils;
    //private PinPointMoveUtils move;

    private DcMotor arm = null; //Located on Expansion Hub- Servo port 0

    private DcMotor lift = null;
    private Servo wrist = null; //Located on Expansion Hub- Servo port 0


    static final float MAX_SPEED = 1.0f;
    static final float MIN_SPEED = 0.4f;
    static final int ACCEL = 75;  // Scaling factor used in accel / decel code.  Was 100!
    public double desiredHeading;
    public Intake intake;
    public SleepAction sleep1Sec;

    Orientation angles;
    Acceleration gravity;


    private boolean done = false;
    private fileUtils fUtils;
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(-60, 15,0);
        drive = new PinpointDrive(hardwareMap, startPose);
        //utils = new actuatorUtils();
        intake = new Intake(hardwareMap);
        sleep1Sec = new SleepAction(1);
        //lift = hardwareMap.get(DcMotor.class, "lift");
        //arm = hardwareMap.get(DcMotor.class, "arm");
        //intake = hardwareMap.get(CRServo.class, "intake");
        //wrist = hardwareMap.get(Servo.class,"wrist");
        //drive.setPoseEstimate(startPose);
        //TrajectorySequence aSeq = autoSeq(startPose);


        //Reverse the arm direction so it moves in the proper direction
        //lift.setDirection(DcMotor.Direction.REVERSE);
        //lift.setPower(0);
        //lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //fUtils = new fileUtils();
        //desiredHeading = getHeading();

        //utils.initializeActuator(lift, arm, intake, wrist);
        //move.initialize(drive, utils, startPose);

       // utils.setArm(actuatorUtils.ArmModes.UP);
        //utils.setIntake(actuatorUtils.IntakeModes.OFF);
        //utils.setWrist(actuatorUtils.WristModes.DOWN);

        Long startTime = System.currentTimeMillis();
        Long currTime = startTime;


        waitForStart();
        currTime = System.currentTimeMillis();
        startTime = currTime;
        //sleep(5000)
        Actions.runBlocking(new ParallelAction(
                new SequentialAction(
                    drive.actionBuilder(startPose)
                        .setTangent(Math.toRadians(0))
                        .splineTo(new Vector2d(-47, 53), Math.toRadians(130))
                        .build(),
                    intake.intakeIn(),
                    new SleepAction(1),
                    intake.intakeOut(),
                    new SleepAction(1),
                    intake.intakeOff()),
                new SequentialAction(
                    intake.intakeOut(),
                    new SleepAction(1),
                    intake.intakeIn(),
                    new SleepAction(1),
                    intake.intakeOff())
                )
        );

        //move.driveSeq(-47,53,130);
       // utils.setLift(actuatorUtils.LiftLevel.HIGH_BASKET);
       // sleep(3000);
        //move.driveSeq(-57,57,135);
        //sleep(1000);
        //utils.setArm(actuatorUtils.ArmModes.UP);

        //telemetry.addData("arm: ",arm.getCurrentPosition());
        //telemetry.update();

        //drop the sample
        //sleep(1000);
        //utils.setLift(actuatorUtils.LiftLevel.HIGH_BASKET);
        //sleep(4000);
        //move.driveSeq(-51,53,130);
        //sleep(1000);
        //utils.setArm(-50);
        //utils.setIntake(actuatorUtils.IntakeModes.OUT);
        //sleep(2000);

        //back up from the baskets
        //utils.setArm(50);
        //move.driveSeq(-36,36,130);
        //sleep(1000);
        //utils.setLift(actuatorUtils.LiftLevel.ZERO);
        //sleep(1000);
        //utils.setArm(actuatorUtils.ArmModes.DOWN);
        //utils.setIntake(actuatorUtils.IntakeModes.OFF);
        //sleep(2000);

        //drive to chamber
        //move.driveSeq(-12,36,-90);
        //utils.setArm(actuatorUtils.ArmModes.REST);
        //utils.setLift(actuatorUtils.LiftLevel.LOW_BASKET);
        //sleep(1000);
        //move.driveSeq(-12,22,-90);
        //sleep(1000);
        //utils.setArm(-200);



        // utils.setArm(actuatorUtils.ArmModes.REST);
        //sleep(1000);
        //Pose2d pose = drive.getPoseEstimate();
        //fUtils.setPose(pose);
        //fUtils.writeConfig(hardwareMap.appContext, this);
        //telemetry.addData("Final Heading: ", "Heading: "+ pose.getHeading());
        //telemetry.update();
    }


    public double getHeading() {
        double angle = drive.pinpoint.getHeading();
        return angle;
    }

}