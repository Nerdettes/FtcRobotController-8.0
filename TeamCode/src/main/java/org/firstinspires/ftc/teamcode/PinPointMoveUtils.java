package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class PinPointMoveUtils {

    private static PinpointDrive drive;
    private static actuatorUtils utils;

    private static Pose2d beginPose;


    public static void initialize(PinpointDrive drive, actuatorUtils utils, Pose2d initialPose){
        PinPointMoveUtils.drive = drive;
        PinPointMoveUtils.utils = utils;
        PinPointMoveUtils.beginPose = initialPose;
    }
    public static void driveSeq (double x, double y, double heading){
        TrajectoryActionBuilder builder = drive.actionBuilder(beginPose);
        if (beginPose.position.x != x)
            builder.lineToX(x);
        if (beginPose.position.y != y)
            builder.lineToY(y);
        if (beginPose.heading.toDouble() != Math.toRadians(heading))
            builder.turn(Math.toRadians(heading));
        Actions.runBlocking( new SequentialAction(
            builder.build()));
        beginPose = new Pose2d(x, y, Math.toRadians(heading));
    }
   /* public static void driveToBoard(double x, double y, double heading) throws InterruptedException{
        utils.elbowBoard();
        utils.armBoard();
        sleep(1000);
        moveUtils.driveSeq(x, y, heading);
        sleep(500);
        utils.gripperOpen();
        sleep(1000);
    }
    public static void driveFromBoard(double x, double y, double heading) throws InterruptedException{
        moveUtils.driveSeq(x,y,heading);
        utils.noArmBoard();
        utils.noElbowBoard();
        sleep(1000);
        utils.gripperClose();
    }
*/



}
