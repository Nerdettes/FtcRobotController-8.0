package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


import java.util.Locale;

@Autonomous(name = "LeftPath", group = "")
public class LeftPath extends LinearOpMode {
    private static final int NUMLOOPS = 3 ;
    //test1

    private SampleMecanumDrive drive;
    private actuatorUtils utils;
    private moveUtils move;

    private DcMotor arm = null; //Located on Expansion Hub- Servo port 0

    private CRServo intake = null; //Located on Expansion Hub- Servo port 0
    private DcMotor lift = null;
    private Servo wrist = null; //Located on Expansion Hub- Servo port 0


    static final float MAX_SPEED = 1.0f;
    static final float MIN_SPEED = 0.4f;
    static final int ACCEL = 75;  // Scaling factor used in accel / decel code.  Was 100!
    public double desiredHeading;

    Orientation angles;
    Acceleration gravity;


    private boolean done = false;
    private fileUtils fUtils;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        utils = new actuatorUtils();
        lift = hardwareMap.get(DcMotor.class, "lift");
        arm = hardwareMap.get(DcMotor.class, "arm");
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist = hardwareMap.get(Servo.class,"wrist");
        Pose2d startPose = new Pose2d(-63, 15,0);
        drive.setPoseEstimate(startPose);
        //TrajectorySequence aSeq = autoSeq(startPose);


        //Reverse the arm direction so it moves in the proper direction
        lift.setDirection(DcMotor.Direction.REVERSE);
        lift.setPower(0);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fUtils = new fileUtils();
        desiredHeading = getHeading();

        utils.initializeActuator(lift, arm, intake, wrist);
        move.initialize(drive, utils);

       // utils.setArm(actuatorUtils.ArmModes.UP);
        utils.setIntake(actuatorUtils.IntakeModes.OFF);
        utils.setWrist(actuatorUtils.WristModes.DOWN);

        Long startTime = System.currentTimeMillis();
        Long currTime = startTime;


        waitForStart();
        currTime = System.currentTimeMillis();
        startTime = currTime;
        //sleep(5000)


        move.driveSeq(startPose.getX()+12,startPose.getY(),0);
        move.driveSeq(-47,53,130);
       // utils.setLift(actuatorUtils.LiftLevel.HIGH_BASKET);
       // sleep(3000);
        //move.driveSeq(-57,57,135);
        sleep(1000);
        utils.setArm(actuatorUtils.ArmModes.UP);

        //telemetry.addData("arm: ",arm.getCurrentPosition());
        //telemetry.update();

        //drop the sample
        sleep(1000);
        utils.setLift(actuatorUtils.LiftLevel.HIGH_BASKET);
        sleep(4000);
        utils.setIntake(actuatorUtils.IntakeModes.OUT);
        sleep(2000);

        //back up from the baskets
        move.driveSeq(-36,36,130);
        sleep(1000);
        utils.setLift(actuatorUtils.LiftLevel.ZERO);
        sleep(1000);
        utils.setArm(actuatorUtils.ArmModes.DOWN);
        utils.setIntake(actuatorUtils.IntakeModes.OFF);
        sleep(2000);

        //drive to chamber
        move.driveSeq(-12,36,-90);
        utils.setArm(actuatorUtils.ArmModes.REST);
        utils.setLift(actuatorUtils.LiftLevel.PARK);
        sleep(1000);
        move.driveSeq(-12,23,-90);


       // utils.setArm(actuatorUtils.ArmModes.REST);
        sleep(1000);
        Pose2d pose = drive.getPoseEstimate();
        fUtils.setPose(pose);
        fUtils.writeConfig(hardwareMap.appContext, this);
        telemetry.addData("Final Heading: ", "Heading: "+ pose.getHeading());
        telemetry.update();
    }




    public double getHeading() {
        double angle = drive.getRawExternalHeading();
        return angle;
    }

}


