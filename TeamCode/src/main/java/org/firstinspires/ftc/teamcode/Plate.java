package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Plate {
    private final DcMotor plate; //Located on Control Hub- Servo port 2

    public Plate(HardwareMap hardwareMap) {
        plate = hardwareMap.get(DcMotor.class, "plate");
        plate.setDirection(DcMotor.Direction.REVERSE);
        plate.setPower(0);
        plate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        plate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        plate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public int getPosition (){
        return plate.getCurrentPosition();
}
    public class LiftRun implements Action {
        private double pow;
        public LiftRun(double pow) { this.pow = pow;}
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            plate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            plate.setPower(pow);
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
            plate.setTargetPosition(pos); //Lowers arm to min pos.
            plate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            plate.setPower(pow);
            if (plate.isBusy()) {
                return true;
            } else {

                return false;
            }
        }
    }
    public Action setPosition(int pos, double pow) {
        return new SetPosition(pos, pow);
    }
    public class SetPositionNoBlock implements Action {
        private int pos;
        private double pow;
        public SetPositionNoBlock(int pos, double pow) {
            this.pos = pos;
            this.pow = pow;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            plate.setTargetPosition(pos); //Lowers arm to min pos.
            plate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            plate.setPower(pow);
            return false;
        }
    }
    public Action setPositionNoBlock(int pos, double pow) {
        return new SetPositionNoBlock(pos, pow);
    }
    public void reset() {
        plate.setPower(0);
        plate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        plate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}
