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

    public Wrist(HardwareMap hardwareMap) {
        leftWrist  = hardwareMap.get(Servo.class, "leftWrist");
        rightWrist = hardwareMap.get(Servo.class, "rightWrist");
        leftWrist.setPosition(1.0/3.0);
        rightWrist.setPosition(1.0 - 1.0/3.0);
    }

    public class WristDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            leftWrist.setPosition(2.0/3.0 + 1.0/27.0);
            rightWrist.setPosition(1.0 - (2.0/3.0 + 1.0/27.0));
            return false;
        }
    }
    public Action wristDown() {
        return new WristDown();
    }

    public class WristUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            leftWrist.setPosition(1.0/3.0 - 1.0/27.0);
            rightWrist.setPosition(1.0 - (1.0/3.0 - 1.0/27.0));
            return false;
        }
    }
    public Action wristUp() {
        return new WristUp();
    }

}

