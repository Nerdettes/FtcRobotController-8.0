package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.SleepAction;


public class KitchenSink {
    public PinpointDrive drive;
    public Intake intake;
    public IntakeSlide intakeSlide;
    public Wrist wrist;
    public GripperWrist gripperWrist;
    public Gripper gripper;
    public Lift lift;

    public KitchenSink(HardwareMap hardwareMap, Pose2d startPose) {
        drive = new PinpointDrive(hardwareMap, startPose);
        intake = new Intake(hardwareMap);
        intakeSlide = new IntakeSlide(hardwareMap);
        wrist = new Wrist(hardwareMap);
        gripperWrist = new GripperWrist(hardwareMap);
        gripper = new Gripper(hardwareMap);
        lift = new Lift(hardwareMap);
    }
    public SequentialAction handoff () {
        return new SequentialAction(
                gripperWrist.wristBack(),
                gripper.gripperOpen(),
                wrist.wristInit(),
                intakeSlide.setPosition(600),
                wrist.wristUp(),
                new SleepAction(1),
                gripperWrist.wristFront(),
                new SleepAction(1),
                intake.intakeOut(),
                gripper.gripperClosed(),
                new SleepAction(1),
                intake.intakeOff(),
                gripperWrist.wristBack()
        );
    }
    public void reset() {
        intakeSlide.reset ();
        lift.reset();

    }
}

