package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.SleepAction;


public class KitchenSink {
    public MecanumDrive drive;
    public Mouth mouth;
    public Throat throat;
    public Plate plate;
    public CookieShooter cookie;

    public KitchenSink(HardwareMap hardwareMap, Pose2d startPose) {
        drive = new MecanumDrive(hardwareMap, startPose);
        mouth = new Mouth(hardwareMap);
        cookie = new CookieShooter(hardwareMap);
        throat = new Throat(hardwareMap);
        //plate = new Plate(hardwareMap);
    }
    public SequentialAction eat () {
        return new SequentialAction(
                throat.ingest(),
                mouth.bite()

        );
    }

    public SequentialAction shoot (double pow) {
        return new SequentialAction(
                cookie.cookie(pow),
                new SleepAction(2.0),
                mouth.bite(),
                throat.ingest()

                );
    }
    public SequentialAction shootFar (double pow) {
        return new SequentialAction(
                cookie.cookie(pow),
                new SleepAction(2.0),
                mouth.bite(),
                throat.ingest()

        );
    }
    public SequentialAction rest () {
        return new SequentialAction(
                throat.digest(),
                mouth.chew()

                );
    }
    public SequentialAction cookieRest () {
        return new SequentialAction(
                rest(),
                cookie.cookie(0)

        );
    }
}

