package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "LeftPath", group = "")
public class LeftPath extends LinearOpMode {
    private static final int NUMLOOPS = 3 ;
    //test1
    private KitchenSink ks;
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
        Pose2d startPose = new Pose2d(-65.25, 15.5,Math.toRadians(180.0));
        ks = new KitchenSink(hardwareMap, startPose);

        //drive.setPoseEstimate(startPose);
        //TrajectorySequence aSeq = autoSeq(startPose);


        //Reverse the arm direction so it moves in the proper direction
        fUtils = new fileUtils();
        desiredHeading = getHeading();
        Long startTime = System.currentTimeMillis();
        Long currTime = startTime;


        waitForStart();
        currTime = System.currentTimeMillis();
        startTime = currTime;
        //sleep(5000)

        Actions.runBlocking(new SequentialAction(
                ks.lift.setPositionNoBlock(1600, 0.5),
                //ks.gripperWrist.wristBack(),
                ks.drive.actionBuilder(startPose)
                        //.setTangent(Math.toRadians(0.0))
                        .strafeToLinearHeading(new Vector2d(-41.375, 3.0), Math.toRadians(180.0))
                        //.splineToConstantHeading(new Vector2d(-41.5, 3.0), Math.toRadians(180.0))
                        //.splineTo(new Vector2d(-39, 3.0), Math.toRadians(180.0))
                        .build(),
                ks.gripperWrist.wristBack(),
                new SleepAction(1),
                ks.lift.setPositionNoBlock(0, 0.5),
                new SleepAction(1),
                ks.gripper.gripperOpen(),
                ks.drive.actionBuilder(new Pose2d(-41.375, 3.0, Math.toRadians(180.0)))
                        .strafeToLinearHeading(new Vector2d(-50, 48.75), Math.toRadians(0.0))
                        .strafeToLinearHeading(new Vector2d(-31.75, 48.75), Math.toRadians(0.0))
                        .build(),
               // new SleepAction(2),
                ks.wrist.wristDown(),
                new SleepAction(1),
                ks.intake.intakeIn(),
                new SleepAction(1),
                ks.intake.intakeOff(),
                ks.handoff(),
                ks.lift.setPosition(1600,0.5),
                ks.drive.actionBuilder(new Pose2d(-31.75, 48.75, Math.toRadians(0.0)))
                        .strafeToLinearHeading(new Vector2d(-52.25, 52.25), Math.toRadians(-45.0))
                        .build(),
                ks.gripper.gripperOpen(),
                new SleepAction(.5),
                ks.wrist.wristInit(),
                ks.intakeSlide.setPositionNoBlock(0),
                ks.drive.actionBuilder(new Pose2d(-52.25, 52.25, Math.toRadians(-45.0)))
                        .strafeToLinearHeading(new Vector2d(-12.5, 48), Math.toRadians(90.0))
                        .strafeToLinearHeading(new Vector2d(-12.5, 28), Math.toRadians(90.0))
                        .build(),
                new SleepAction(1),
                ks.gripperWrist.wristWall()
                )
        );
        // utils.setArm(actuatorUtils.ArmModes.REST);
        telemetry.addData("IntakeSlide Position: ", ks.intakeSlide.getPosition());
        telemetry.update();
        //Pose2d pose = drive.getPoseEstimate();
        //fUtils.setPose(pose);
        //fUtils.writeConfig(hardwareMap.appContext, this);
    }


    public double getHeading() {
        double angle = ks.drive.pinpoint.getHeading();
        return angle;
    }

}