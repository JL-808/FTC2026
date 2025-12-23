package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    private DcMotorEx liftLeft;
    private DcMotorEx liftRight;

    public Lift(HardwareMap hardwareMap) {
        liftLeft = hardwareMap.get(DcMotorEx.class, "lift_left");
        liftRight = hardwareMap.get(DcMotorEx.class, "lift_right");

        liftLeft.setDirection(DcMotorEx.Direction.REVERSE);
        liftRight.setDirection(DcMotorEx.Direction.FORWARD);


        liftLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public class LiftUp implements Action {
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                liftLeft.setPower(1);
                liftRight.setPower(1);
                initialized = true;
            }

            // checks lift's current position
            double pos = liftLeft.getCurrentPosition();
            packet.put("liftPos", pos);
            if (pos < 3000.0) {
                // true causes the action to rerun
                return true;
            } else {
                // false stops action rerun
                liftLeft.setPower(0);
                liftRight.setPower(0);
                return false;
            }
            // overall, the action powers the lift until it surpasses
            // 3000 encoder ticks, then powers it off
        }
    }
}
