
package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.*;
@Disabled
public final class ManualFeedbackTuner extends LinearOpMode {
    public static double DISTANCE = 64;

    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(PinpointDrive.class)) {
            PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();
            boolean yIsPressed = true;
            boolean xIsPressed = false;
            boolean aIsPressed = false;
            boolean bIsPressed = false;
            while (opModeIsActive()) {
                if (gamepad1.y || (yIsPressed && !gamepad1.x && !gamepad1.a && !gamepad1.b)) {
                    yIsPressed = true;
                    xIsPressed = false;
                    aIsPressed = false;
                    bIsPressed = false;
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(0, 0, 0))
                                    .lineToX(DISTANCE)
                                    .lineToX(0)
                                    .build());
                } else if (gamepad1.x || (xIsPressed && !gamepad1.y && !gamepad1.a && !gamepad1.b)) {
                    yIsPressed = false;
                    xIsPressed = true;
                    aIsPressed = false;
                    bIsPressed = false;
                    Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(0, 0, 0))
                                    .strafeTo(new Vector2d(0, DISTANCE))
                                    .strafeTo(new Vector2d(0, 0))
                                    .build());
                } else if (gamepad1.a || (aIsPressed && !gamepad1.y && !gamepad1.x && !gamepad1.b)) {
                    yIsPressed = false;
                    xIsPressed = false;
                    aIsPressed = true;
                    bIsPressed = false;
                    Pose2d pose = drive.pinpoint.getPositionRR();
                    telemetry.addData("X      :", pose.position.x);
                    telemetry.addData("Y      :", pose.position.y);
                    telemetry.addData("Heading: ", Math.toDegrees(pose.heading.toDouble()));
                    telemetry.update();
                    Actions.runBlocking(
                            drive.actionBuilder(pose)
                                    .setTangent(0)
                                    .lineToXSplineHeading(DISTANCE, Math.toRadians(180))
                                    .setTangent(Math.toRadians(180))
                                    .lineToXSplineHeading(0, Math.toRadians(0))
                                    .build());
                    pose = drive.pinpoint.getPositionRR();
                    telemetry.addData("X      :", pose.position.x);
                    telemetry.addData("Y      :", pose.position.y);
                    telemetry.addData("Heading: ", Math.toDegrees(pose.heading.toDouble()));
                    telemetry.update();
                } else if (gamepad1.b || (bIsPressed && !gamepad1.y && !gamepad1.x && !gamepad1.a)) {
                    yIsPressed = false;
                    xIsPressed = false;
                    aIsPressed = false;
                    bIsPressed = true;
                }
            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(SparkFunOTOSDrive.class)) {
            SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {
                Actions.runBlocking(
                        drive.actionBuilder(new Pose2d(0, 0, 0))
                                .lineToX(DISTANCE)
                                .lineToX(0)
                                .build());
            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            if (drive.localizer instanceof TwoDeadWheelLocalizer) {
                if (TwoDeadWheelLocalizer.PARAMS.perpXTicks == 0 && TwoDeadWheelLocalizer.PARAMS.parYTicks == 0) {
                    throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
                }
            } else if (drive.localizer instanceof ThreeDeadWheelLocalizer) {
                if (ThreeDeadWheelLocalizer.PARAMS.perpXTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par0YTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par1YTicks == 1) {
                    throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
                }
            }
            waitForStart();

            while (opModeIsActive()) {
                Actions.runBlocking(
                        drive.actionBuilder(new Pose2d(0, 0, 0))
                                .lineToX(DISTANCE)
                                .lineToX(0)
                                .build());
            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

            if (drive.localizer instanceof TwoDeadWheelLocalizer) {
                if (TwoDeadWheelLocalizer.PARAMS.perpXTicks == 0 && TwoDeadWheelLocalizer.PARAMS.parYTicks == 0) {
                    throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
                }
            } else if (drive.localizer instanceof ThreeDeadWheelLocalizer) {
                if (ThreeDeadWheelLocalizer.PARAMS.perpXTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par0YTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par1YTicks == 1) {
                    throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
                }
            }
            waitForStart();

            while (opModeIsActive()) {
                Actions.runBlocking(
                        drive.actionBuilder(new Pose2d(0, 0, 0))
                                .lineToX(DISTANCE)
                                .lineToX(0)
                                .build());
            }
        } else {
            throw new RuntimeException();
        }
    }
}
