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
@TeleOp(name = "Ball Launch Test", group = "Main")
public class BallLaunchTest extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    public static double ballVelocity = 2000;


    private BallLaunch ballLaunch;

    private TelemetryManager telemetryM;

    @Override
    public void init() {
        ballLaunch = new BallLaunch(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
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
        if (gamepad1.dpad_up || gamepad2.x) {
            ballLaunch.forceLaunch = true;
            ballLaunch.setTargetVelocity(ballVelocity);
        } else {
            ballLaunch.forceLaunch = false;
        }

        ballLaunch.update();

        if (gamepad1.a) {
            ballLaunch.launch();
        }


        telemetryM.addData("Velocity", ballLaunch.getVelocity());
        telemetryM.addData("Target Velocity", ballLaunch.getTargetVelocity());
        telemetryM.update(telemetry);
    }

    @Override
    public void stop() {
    }
}
