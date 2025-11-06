package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "RedCookiePath2", group = "")
public class RedCookiePath2 extends LinearOpMode {
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
        Pose2d startPose = new Pose2d(24.0, -69.0,Math.toRadians(-90.0));
        ks = new KitchenSink(hardwareMap, startPose);

        //drive.setPoseEstimate(startPose);
        //TrajectorySequence aSeq = autoSeq(startPose);


        //Reverse the arm direction so it moves in the proper direction
        // fUtils = new fileUtils();
        // desiredHeading = getHeading();
        Long startTime = System.currentTimeMillis();
        Long currTime = startTime;


        waitForStart();
        currTime = System.currentTimeMillis();
        startTime = currTime;
        //sleep(5000)
        Actions.runBlocking(new SequentialAction(
                ks.drive.actionBuilder(startPose)
                        .setTangent(Math.toRadians(-90.0))
                        .strafeToLinearHeading(new Vector2d(12.0, 18.0), Math.toRadians(120.0))
                        .setTangent(Math.toRadians(120.0))
                        .strafeToLinearHeading(new Vector2d(24.0, 30.0), Math.toRadians(225.0))
                        .build(),
                ks.shoot(0.5),
                new SleepAction(1.0),
                ks.cookieRest(),
                ks.drive.actionBuilder(new Pose2d(new Vector2d(24.0, 30.0), Math.toRadians(224.0)))
                        .setTangent(Math.toRadians(225.0))
                        .strafeToLinearHeading(new Vector2d(24.0, -36.0), Math.toRadians(0.0))
                        .build(),
                ks.eat(),
                ks.drive.actionBuilder(new Pose2d(new Vector2d(24.0, -36.0), Math.toRadians(0.0)))
                        .setTangent(Math.toRadians(0.0))
                        .strafeToLinearHeading(new Vector2d(54.0, -36.0), Math.toRadians(0.0))
                        .strafeToLinearHeading(new Vector2d(24.0, -36.0), Math.toRadians(225.0))
                        .strafeToLinearHeading(new Vector2d(24.0, 30.0), Math.toRadians(225.0))
                        .build(),
                ks.rest(),
                ks.shoot(0.5),
                new SleepAction(1.0),
                ks.cookieRest(),
                ks.drive.actionBuilder(new Pose2d(new Vector2d(24.0, 30.0), Math.toRadians(225.0)))
                        .setTangent(Math.toRadians(225.0))
                        .strafeToLinearHeading(new Vector2d(15.0, -36.0), Math.toRadians(90.0))
                        .build()
        ));


        // utils.setArm(actuatorUtils.ArmModes.REST);
        //telemetry.addData("IntakeSlide Position: ", ks.throat.getPosition());
        // telemetry.update();
        //Pose2d pose = drive.getPoseEstimate();
        //fUtils.setPose(pose);
        //fUtils.writeConfig(hardwareMap.appContext, this);
    }


    public double getHeading() {
        // double angle = ks.drive.pinpoint.getHeading();
        return 0.0;
    }

}