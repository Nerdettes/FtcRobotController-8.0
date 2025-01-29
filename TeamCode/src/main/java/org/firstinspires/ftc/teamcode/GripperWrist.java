package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class GripperWrist {
    private final Servo wrist; //Located on Control Hub- Servo port 2

    private final double backPosition = (0.0/270.0);
    private final double frontPosition   = (190.0/270.0);
    public GripperWrist(HardwareMap hardwareMap) {
        wrist  = hardwareMap.get(Servo.class, "gripperWrist");
        wrist.setPosition(frontPosition);
    }

    public class WristFront implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            wrist.setPosition(frontPosition);
            return false;
        }
    }
    public Action wristFront() {
        return new WristFront();
    }

    public class WristBack implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            wrist.setPosition(backPosition);
            return false;
        }
    }
    public Action wristBack() {
        return new WristBack();
    }

}

