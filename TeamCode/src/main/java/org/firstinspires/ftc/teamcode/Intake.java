package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.CRServo;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public class Intake {
    private final CRServo intakeLeft; //Located on Control Hub- Servo port 2
    private final CRServo intakeRight; //Located on Control Hub- Servo port 2

    public Intake(HardwareMap hardwareMap) {
        intakeLeft  = hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");
        intakeLeft.setPower(0.0);
        intakeRight.setPower(0.0);
    }

    public class IntakeIn implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeLeft.setPower(-1.0);
            intakeRight.setPower(1.0);
            return false;
        }
    }
    public Action intakeIn() {
        return new IntakeIn();
    }

    public class IntakeOut implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeLeft.setPower(1.0);
            intakeRight.setPower(-1.0);
            return false;
        }
    }
    public Action intakeOut() {
        return new IntakeOut();
    }

    public class IntakeOff implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeLeft.setPower(0.0);
            intakeRight.setPower(0.0);
            return false;
        }
    }
    public Action intakeOff() {
        return new IntakeOff();
    }

    public class IntakeRun implements Action {
        private double pow;
        public IntakeRun(double pow) { this.pow = pow;}
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeLeft.setPower(pow);
            intakeRight.setPower(-pow);
            return false;
        }
    }
    public Action intakeRun(double pow) {
        return new IntakeRun(pow);
    }
}
