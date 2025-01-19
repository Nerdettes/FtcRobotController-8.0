package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public class IntakeSlide {
    private final DcMotor slide; //Located on Control Hub- Servo port 2

    public IntakeSlide(HardwareMap hardwareMap) {
        slide = hardwareMap.get(DcMotor.class, "intakeSlide");
        slide.setDirection(DcMotor.Direction.FORWARD);
        slide.setPower(0);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public class IntakeRun implements Action {
        private double pow;
        public IntakeRun(double pow) { this.pow = pow;}
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            slide.setPower(pow);
            return false;
        }
    }
    public Action intakeRun(double pow) {
        return new IntakeRun(pow);
    }
}
