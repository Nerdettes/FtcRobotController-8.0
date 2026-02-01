package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class CookieShooter {
    private final DcMotor [] shooter = new DcMotor[2]; //Located on Control Hub- Servo port 2
    private VoltageSensor hubVoltage;
    public CookieShooter(HardwareMap hardwareMap) {
        shooter[0] = hardwareMap.get(DcMotor.class, "SR");
        shooter[1] = hardwareMap.get(DcMotor.class, "SL");
        hubVoltage = hardwareMap.get(VoltageSensor.class, "Control Hub");
        shooter[0].setDirection(DcMotor.Direction.FORWARD);
        shooter[0].setPower(0);
        shooter[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter[1].setDirection(DcMotor.Direction.REVERSE);
        shooter[1].setPower(0);
        shooter[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public class Cookie implements Action {
        private double pow;
        public Cookie(double pow) { this.pow = pow;}
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double voltage = hubVoltage.getVoltage();
            //double newPow = 13.25 / voltage * pow;
            double newPow = 13.25 / voltage * pow;
            shooter[0].setPower(newPow);
            shooter[1].setPower(newPow);
            return false;
        }
    }
    public Action cookie(double pow) {
        return new Cookie(pow);
    }

    public void reset() {
        shooter[0].setPower(0);
        shooter[1].setPower(0);

    }
}
