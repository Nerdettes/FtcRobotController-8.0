package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.CRServo;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public class Throat {
    private final CRServo throat1; //Located on Control Hub- Servo port 1
    private final CRServo throat2; //Located on Control Hub- Servo port 2

    public Throat(HardwareMap hardwareMap) {
        throat1  = hardwareMap.get(CRServo.class, "throat1");
        throat1.setPower(0.0);
        throat2  = hardwareMap.get(CRServo.class, "throat2");
        throat2.setPower(0.0);
    }

    public class Ingest implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            throat1.setPower(-1.0);
            throat2.setPower(-1.0);
            return false;
        }
    }
    public Action ingest() {
        return new Ingest();
    }

    public class UpChuck implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            throat1.setPower(1.0);
            throat2.setPower(1.0);
            return false;
        }
    }
    public Action upChuck() {
        return new UpChuck();
    }

    public class Digest implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            throat1.setPower(0.0);
            throat2.setPower(0.0);
            return false;
        }
    }
    public Action digest() {
        return new Digest();
    }

}
