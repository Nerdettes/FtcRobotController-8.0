package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gripper {
    private final Servo gripper; //Located on Control Hub- Servo port 2

    private final double openPosition = (0.08);
    private final double closedPosition   = (0.01);
    public Gripper(HardwareMap hardwareMap) {
        gripper  = hardwareMap.get(Servo.class, "gripper");
        gripper.setPosition(closedPosition);
    }

    public class GripperClosed implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            gripper.setPosition(closedPosition);
            return false;
        }
    }
    public Action gripperClosed() {
        return new GripperClosed ();
    }

    public class GripperOpen implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            gripper.setPosition(openPosition);
            return false;
        }
    }
    public Action gripperOpen() {
        return new GripperOpen();
    }

}

