package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public class Throat {
    private final DcMotor throat; //Located on Control Hub- Servo port 2

    public Throat(HardwareMap hardwareMap) {
        throat = hardwareMap.get(DcMotor.class, "throat");
        throat.setDirection(DcMotor.Direction.REVERSE);
        throat.setPower(0);
        throat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        throat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        throat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public int getPosition (){
        return throat.getCurrentPosition();
}
    public class Cookie implements Action {
        private double pow;
        public Cookie(double pow) { this.pow = pow;}
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            throat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            throat.setPower(pow);
            return false;
        }
    }
    public Action cookie(double pow) {
        return new Cookie(pow);
    }
    public class SetPosition implements Action {
        private int pos;
        public SetPosition(int pos) { this.pos = pos;}
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            throat.setTargetPosition(pos); //Lowers arm to min pos.
            throat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            throat.setPower(0.5);
            if (throat.isBusy()) {
                return true;
            } else {
                throat.setPower(0.0);
                throat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                return false;
            }
        }
    }
    public Action setPosition(int pos) {
        return new SetPosition(pos);
    }
    public class SetPositionNoBlock implements Action {
        private int pos;
        public SetPositionNoBlock(int pos) { this.pos = pos;}
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            throat.setTargetPosition(pos); //Lowers arm to min pos.
            throat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            throat.setPower(0.5);
            return false;
        }
    }
    public Action setPositionNoBlock(int pos) {
    return new SetPositionNoBlock(pos);
}
    public void reset() {
        throat.setPower(0);
        throat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        throat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}
