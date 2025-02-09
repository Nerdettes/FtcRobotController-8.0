package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    private final DcMotor lift; //Located on Control Hub- Servo port 2

    public Lift(HardwareMap hardwareMap) {
        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setDirection(DcMotor.Direction.REVERSE);
        lift.setPower(0);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public int getPosition (){
        return lift.getCurrentPosition();
}
    public class LiftRun implements Action {
        private double pow;
        public LiftRun(double pow) { this.pow = pow;}
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setPower(pow);
            return false;
        }
    }
    public Action liftRun(double pow) {
        return new LiftRun(pow);
    }
    public class SetPosition implements Action {
        private int pos;
        private double pow;
        public SetPosition(int pos, double pow) {
            this.pos = pos;
            this.pow = pow;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            lift.setTargetPosition(pos); //Lowers arm to min pos.
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(pow);
            if (lift.isBusy()) {
                return true;
            } else {

                return false;
            }
        }
    }
    public Action setPosition(int pos, double pow) {
        return new SetPosition(pos, pow);
    }
    public void reset() {
        lift.setPower(0);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}
