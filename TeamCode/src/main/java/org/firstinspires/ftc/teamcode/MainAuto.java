package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous (name="Main Auto", group="Main")

public class MainAuto extends OpMode {
    private final FtcDashboard dash = FtcDashboard.getInstance();
    private MecanumDrive drive;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        telemetry.addData("Status", "Initializing");
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Alliance Color", "Red");
        telemetry.addData("Start Position", "0");
        telemetry.update();

        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, initialPose);
    }

    @Override
    public void init_loop() {
        if (gamepad1.bWasPressed()) {
            Globals.isRed = true;
            telemetry.addData("Alliance Color", "Red");
        }
        if (gamepad1.xWasPressed()) {
            Globals.isRed = false;
            telemetry.addData("Alliance Color", "Blue");
        }
        if (gamepad1.dpadUpWasPressed()) {
            Globals.startPosition = 0;
            telemetry.addData("Start Position", "0");
        }
        if (gamepad1.dpadDownWasPressed()) {
            Globals.startPosition = 1;
            telemetry.addData("Start Position", "1");
        }
        telemetry.update();
    }

    @Override
    public void start() {
        if (Globals.startPosition == 0) {
            if (Globals.isRed) {
                drive.localizer.setPose(new Pose2d(11.8, 61.7, Math.toRadians(90)));
            } else {
                drive.localizer.setPose(new Pose2d(11.8, -61.7, Math.toRadians(-90)));
            }
        } else {
            if (Globals.isRed) {
                drive.localizer.setPose(new Pose2d(-11.8, 61.7, Math.toRadians(90)));
            } else {
                drive.localizer.setPose(new Pose2d(-11.8, -61.7, Math.toRadians(-90)));
            }
        }
    }


    @Override
    public void loop() {


    }

    @Override
    public void stop() {
        Pose2d pose = drive.localizer.getPose();

        Globals.PoseX = pose.position.x;
        Globals.PoseY = pose.position.y;
        Globals.PoseHeading = pose.heading.toDouble();
    }
}
