package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {
    private final Servo leftWrist; //Located on Control Hub- Servo port 2
    private final Servo rightWrist; //Located on Control Hub- Servo port 4

    private final double downPosition = (190.0/270.0);
    private final double upPosition   = (13.5/270.0);
    private final double initPosition = (90.0/270.0);
    public Wrist(HardwareMap hardwareMap) {
        leftWrist  = hardwareMap.get(Servo.class, "leftWrist");
        rightWrist = hardwareMap.get(Servo.class, "rightWrist");
        leftWrist.setPosition(initPosition);
        rightWrist.setPosition(1.0 - initPosition);
    }

    public class WristDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            leftWrist.setPosition(downPosition);
            rightWrist.setPosition(1.0 - downPosition);
            return false;
        }
    }
    public Action wristDown() {
        return new WristDown();
    }

    public class WristUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            leftWrist.setPosition(upPosition);
            rightWrist.setPosition(1.0 - upPosition);
            return false;
        }
    }
    public Action wristUp() {
        return new WristUp();
    }

    public class WristInit implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            leftWrist.setPosition(initPosition);
            rightWrist.setPosition(1.0 - initPosition);
            return false;
        }
    }
    public Action wristInit() {
        return new WristInit();
    }

}

