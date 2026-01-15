package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
@TeleOp(name = "Lift Test", group = "Main")
public class LiftTest extends OpMode {
    private final FtcDashboard dash = FtcDashboard.getInstance();
    private final ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx liftLeft;
    private DcMotorEx liftRight;

    @Override
    public void init() {
        liftLeft = hardwareMap.get(DcMotorEx.class, "lift_left");
        liftRight = hardwareMap.get(DcMotorEx.class, "lift_right");



        // CHANGE
        liftLeft.setDirection(DcMotor.Direction.REVERSE);
        liftRight.setDirection(DcMotor.Direction.REVERSE);





        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        TelemetryPacket packet = new TelemetryPacket();


        if (gamepad2.dpad_up && !gamepad2.dpad_down) {
            liftLeft.setPower(0.75);
            liftRight.setPower(0.75);
        } else if (gamepad2.dpad_down && !gamepad2.dpad_up) {
            liftLeft.setPower(-0.1);
            liftRight.setPower(-0.1);
        } else {
            liftLeft.setPower(0);
            liftRight.setPower(0);
        }



        telemetry.addData("Run Time", runtime);

        telemetry.addData("left lift Position", liftLeft.getCurrentPosition());
        telemetry.addData("right lift Position", liftRight.getCurrentPosition());
        telemetry.addData("lift power left/Right", "%4.2f, %4.2f", liftLeft.getPower(), liftRight.getPower());

        telemetry.update();

        dash.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
    }
}
