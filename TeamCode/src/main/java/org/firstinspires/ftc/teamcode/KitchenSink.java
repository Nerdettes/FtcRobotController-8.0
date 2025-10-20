package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.SleepAction;


public class KitchenSink {
    public MecanumDrive drive;
    public Mouth mouth;
    public Throat throat;
    public Wrist wrist;
    public GripperWrist gripperWrist;
    public Gripper gripper;
    public Plate plate;

    public KitchenSink(HardwareMap hardwareMap, Pose2d startPose) {
        drive = new MecanumDrive(hardwareMap, startPose);
        //mouth = new Mouth(hardwareMap);
        //throat = new Throat(hardwareMap);
        //wrist = new Wrist(hardwareMap);
        //gripperWrist = new GripperWrist(hardwareMap);
        //gripper = new Gripper(hardwareMap);
        //plate = new Plate(hardwareMap);
    }
    public SequentialAction handoff () {
        return new SequentialAction(
                gripperWrist.wristBack(),
                gripper.gripperOpen(),
                wrist.wristInit(),
                throat.setPosition(600),
                wrist.wristUp(),
                new SleepAction(1),
                gripperWrist.wristFront(),
                new SleepAction(1),
                mouth.upChuck(),
                new SleepAction(0.125),
                gripper.gripperClosed(),
                new SleepAction(1),
                mouth.chew(),
                gripperWrist.wristBack()
        );
    }
    public void reset() {
        throat.reset ();
        plate.reset();

    }
}

