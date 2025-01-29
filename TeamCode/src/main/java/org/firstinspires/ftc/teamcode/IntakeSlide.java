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
        slide.setDirection(DcMotor.Direction.REVERSE);
        slide.setPower(0);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public int getPosition (){
        return slide.getCurrentPosition();
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
    public class SetPosition implements Action {
        private int pos;
        public SetPosition(int pos) { this.pos = pos;}
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            slide.setTargetPosition(pos); //Lowers arm to min pos.
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(0.5);
            if (slide.isBusy()) {
                return true;
            } else {
                slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                return false;
            }
        }
    }
    public Action setPosition(int pos) {
        return new SetPosition(pos);
    }
}
