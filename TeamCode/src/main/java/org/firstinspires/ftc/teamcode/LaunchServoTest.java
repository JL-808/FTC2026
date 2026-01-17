package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Configurable
@TeleOp(name = "Launch Servo Test", group = "Main")
public class LaunchServoTest extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    public static double servoStartPosition = 0.8;
    public static double servoLaunchPosition = 0.5;


    private Servo launchServo;

    @Override
    public void init() {
        launchServo = hardwareMap.get(Servo.class, "launch");
        launchServo.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            launchServo.setPosition(servoLaunchPosition);
        } else {
            launchServo.setPosition(servoStartPosition);
        }
        telemetry.addData("Servo Position", launchServo.getPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}
