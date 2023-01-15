package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public final class RawEncoder implements Encoder {
    private final DcMotorEx m;
    public DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;

    public RawEncoder(DcMotorEx m) {
        this.m = m;
    }

    private int applyDirection(int x) {
        // This is already done by the DcMotorEx routines.  This is undoing what it is doing
        //if (m.getDirection() == DcMotorSimple.Direction.REVERSE) {
        //    x = -x;
        //}

        if (direction == DcMotorSimple.Direction.REVERSE) {
            x = -x;
        }

        return x;
    }

    @Override
    public PositionVelocityPair getPositionAndVelocity() {
        return new PositionVelocityPair(
                applyDirection(m.getCurrentPosition()),
                applyDirection((int) m.getVelocity())
        );
    }

    @Override
    public DcMotorController getController() {
        return m.getController();
    }
}
