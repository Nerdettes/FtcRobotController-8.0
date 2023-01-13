package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.List;

public final class ForwardPushTest extends LinearOpMode {
    private static double avgPos(List<? extends Encoder> es) {
        double avgPos = 0;
        for (Encoder e : es) {
            avgPos += e.getPositionAndVelocity().position;
        }
        return avgPos / es.size();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        DriveView view = new DriveView(hardwareMap);

        for (DcMotorEx m : view.motors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        waitForStart();

        double initAvgPos = avgPos(view.forwardEncs);
        while (opModeIsActive()) {
            telemetry.addData("Using getPositionAndVelocity().position","");
            for (Encoder e : view.forwardEncs) {
                telemetry.addData(e.toString(), e.getPositionAndVelocity().position);
            }
            telemetry.addData("Using getCurrentPosition()","");
            telemetry.addData("LeftFront", view.md.leftFront.getCurrentPosition());
            telemetry.addData("RightFront", view.md.rightFront.getCurrentPosition());
            telemetry.addData("LeftBack", view.md.leftBack.getCurrentPosition());
            telemetry.addData("RightBack", view.md.rightBack.getCurrentPosition());
            telemetry.addData("ticks traveled", avgPos(view.forwardEncs) - initAvgPos);

            telemetry.update();
        }
    }
}
