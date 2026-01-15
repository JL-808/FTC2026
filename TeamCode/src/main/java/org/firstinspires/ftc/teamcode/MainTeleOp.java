package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "Main TeleOp", group = "Main")
public class MainTeleOp extends OpMode {
    private final FtcDashboard dash = FtcDashboard.getInstance();
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor intake;
    private DcMotorEx outtake;

    private Servo launchServo;

    private MecanumDrive drive;

    private Lift lift;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        telemetry.addData("Status", "Initializing");

        intake = hardwareMap.get(DcMotor.class, "intake");

        intake.setDirection(DcMotor.Direction.REVERSE);

        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake.setDirection(DcMotor.Direction.FORWARD);

        lift = new Lift(hardwareMap);

        launchServo = hardwareMap.get(Servo.class, "launch");
        launchServo.setDirection(Servo.Direction.REVERSE);


        drive = new MecanumDrive(hardwareMap, new Pose2d(Globals.PoseX, Globals.PoseY, Globals.PoseHeading));
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
        lift.reset();
    }


    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        if (gamepad1.left_trigger > 0.05 && gamepad1.right_trigger <= 0.05) { // Strafe left using left trigger
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            0,
                            gamepad1.left_trigger / 1.5
                    ),
                    0
            ));

        } else if (gamepad1.right_trigger > 0.05 && gamepad1.left_trigger <= 0.05) { // Strafe right using right trigger
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            0,
                            -gamepad1.right_trigger / 1.5
                    ),
                    0
            ));
        } else if (gamepad1.left_trigger <= 0.05 && gamepad1.right_trigger <= 0.05) { // Normal driving
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));
        } else { // Both triggers are pressed, normal driving
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));
        }

        drive.updatePoseEstimate();

        Pose2d pose = drive.localizer.getPose();

        if (gamepad2.dpad_right) {
            intake.setPower(0.75);
        } else if (gamepad2.dpad_left) {
            intake.setPower(-0.75);
        } else {
            intake.setPower(0.0);
        }

        if (gamepad1.dpadUpWasPressed()) {
            outtake.setPower(1.0);
        }
        if (gamepad1.dpadUpWasReleased()) {
            outtake.setPower(0.0);
        }

        if (gamepad2.dpad_up && !gamepad2.dpad_down) {
            lift.up();
        } else if (gamepad2.dpad_down && !gamepad2.dpad_up) {
            lift.down();
        }

        lift.update(); // Update lift control loop


        final double LAUNCH_SERVO_EXTENDED = 0.7;
        final double LAUNCH_SERVO_RETRACTED = 0.31;

        if (gamepad1.aWasPressed()) {
            launchServo.setPosition(LAUNCH_SERVO_EXTENDED);
        } else if (gamepad1.aWasReleased()) {
            launchServo.setPosition(LAUNCH_SERVO_RETRACTED);
        }


        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), pose);

        telemetry.addData("Run Time", runtime);
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", drive.leftFront.getPower(), drive.rightFront.getPower());
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", drive.leftBack.getPower(), drive.rightBack.getPower());
        telemetry.addData("Target Ticks", lift.targetTicks);
        telemetry.addData("left lift Position", lift.liftLeft.getCurrentPosition());
        telemetry.addData("right lift Position", lift.liftRight.getCurrentPosition());
        telemetry.addData("lift power left/Right", "%4.2f, %4.2f", lift.liftLeft.getPower(), lift.liftRight.getPower());
        telemetry.addData("Outtake velocity (tick/s)", outtake.getVelocity());

        telemetry.addData("Pose X", "%.2f", pose.position.x);
        telemetry.addData("Pose Y", "%.2f", pose.position.y);
        telemetry.addData("Pose Heading", "%.2f", Math.toDegrees(pose.heading.toDouble()));
        telemetry.update();

        dash.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
    }
}
