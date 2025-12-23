package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class BallLaunch {
    public static String OUTTAKE_MOTOR_NAME = "outtake";
    private DcMotorEx outtake;
    private Telemetry telemetry;

    public BallLaunch(HardwareMap hardwareMap) {
        outtake = hardwareMap.get(DcMotorEx.class, OUTTAKE_MOTOR_NAME);
        outtake.setDirection(DcMotor.Direction.FORWARD);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public class SpinUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                outtake.setVelocity(10000);
                initialized = true;
            }



            double vel = outtake.getVelocity();
            packet.put("outtake", vel);
            return vel < 10_000.0;
        }
    }

    public Action spinUp() {
        return new SpinUp();
    }


    public int try_launch() {
        // check if the motor is at correct speed

        launch();

        return 0; // success
    }

    public void launch() {
        // launch
    }

    public void start() {
        outtake.setPower(1.0);
    }

    public void stop() {
        outtake.setPower(0.0);
    }

    public int calculatePower(double distance) {
        // kinematics?

        return 0;
    }
}
