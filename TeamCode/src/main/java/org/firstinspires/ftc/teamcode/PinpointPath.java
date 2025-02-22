package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.List;

@Disabled
@Autonomous(name = "PinpointPath", group = "")
public class PinpointPath extends LinearOpMode {
    private static final int NUMLOOPS = 3 ;
    //test1

    private PinpointDrive drive;
    private actuatorUtils utils;
    private PinPointMoveUtils move;

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
    //private fileUtils fUtils;
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(-48, 48,Math.toRadians(0));
        drive = new PinpointDrive(hardwareMap, startPose);
        utils = new actuatorUtils();
        //lift = hardwareMap.get(DcMotor.class, "lift");
        //arm = hardwareMap.get(DcMotor.class, "arm");
        //intake = hardwareMap.get(CRServo.class, "intake");
        //wrist = hardwareMap.get(Servo.class,"wrist");

        //TrajectorySequence aSeq = autoSeq(startPose);


        //Reverse the arm direction so it moves in the proper direction
        //lift.setDirection(DcMotor.Direction.REVERSE);
        //lift.setPower(0);
        //lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //fUtils = new fileUtils();
        //desiredHeading = getHeading();

        utils.initializeActuator(lift, arm, intake, wrist);
        move.initialize(drive, utils, startPose);

       // utils.setArm(actuatorUtils.ArmModes.UP);
        //utils.setIntake(actuatorUtils.IntakeModes.OFF);
        //utils.setWrist(actuatorUtils.WristModes.DOWN);

        Long startTime = System.currentTimeMillis();
        Long currTime = startTime;


        waitForStart();
        currTime = System.currentTimeMillis();
        startTime = currTime;
        //sleep(5000)

        while (opModeIsActive()) {
            Actions.runBlocking(drive.actionBuilder(startPose)
                    .setTangent(Math.toRadians(0))
                    .lineToXSplineHeading(48, Math.toRadians(-90))
                    .setTangent(Math.toRadians(-90))
                    .lineToYSplineHeading(-48, Math.toRadians(180))
                    .setTangent(Math.toRadians(180))
                    .lineToXSplineHeading(-48, Math.toRadians(90))
                    .setTangent(Math.toRadians(90))
                    .lineToYSplineHeading(48, Math.toRadians(0))
                    .build()
            );
        }
    }


    public double getHeading() {
        double angle = drive.pinpoint.getHeading();
        return angle;
    }

}