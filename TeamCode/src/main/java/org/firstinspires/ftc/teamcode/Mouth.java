package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.CRServo;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public class Mouth {
    private final CRServo intake; //Located on Control Hub- Servo port 2

    public Mouth(HardwareMap hardwareMap) {
        intake  = hardwareMap.get(CRServo.class, "intake");
        intake.setPower(0.0);
    }

    public class Bite implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake.setPower(-1.0);
            return false;
        }
    }
    public Action bite() {
        return new Bite();
    }

    public class UpChuck implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake.setPower(1.0);
            return false;
        }
    }
    public Action upChuck() {
        return new UpChuck();
    }

    public class Chew implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake.setPower(0.0);
            return false;
        }
    }
    public Action chew() {
        return new Chew();
    }

    public class Cookie implements Action {
        private double pow;
        public Cookie(double pow) { this.pow = pow;}
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake.setPower(pow);
            return false;
        }
    }
    public Action cookie(double pow) {
        return new Cookie(pow);
    }
}
