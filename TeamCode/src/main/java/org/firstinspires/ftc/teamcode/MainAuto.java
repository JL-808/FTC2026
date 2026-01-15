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
        telemetry.addData("Alliance Colour", "Red");
        telemetry.addData("Start Position", "0");
        telemetry.update();

        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, initialPose);
    }

    @Override
    public void init_loop() {
        if (gamepad1.bWasPressed()) {
            Globals.isRed = true;
        }
        if (gamepad1.xWasPressed()) {
            Globals.isRed = false;
        }
        if (gamepad1.dpadUpWasPressed()) {
            Globals.startPosition = 0;
        }
        if (gamepad1.dpadDownWasPressed()) {
            Globals.startPosition = 1;
        }
        if (Globals.startPosition == 0) {
            telemetry.addData("Start Position", "0");
        } else {
            telemetry.addData("Start Position", "1");
        }
        if (Globals.isRed) {
            telemetry.addData("Alliance Colour", "Red");
        } else {
            telemetry.addData("Alliance Colour", "Blue");
        }
        telemetry.update();
    }

    @Override
    public void start() {
        if (Globals.startPosition == 0) {
            if (Globals.isRed) {
                drive.localizer.setPose(new Pose2d(0, 0, Math.toRadians(0)));
            } else {
                drive.localizer.setPose(new Pose2d(0, 0, Math.toRadians(0)));
            }
        } else {
            if (Globals.isRed) {
                drive.localizer.setPose(new Pose2d(62, 13, Math.toRadians(180)));
            } else {
                drive.localizer.setPose(new Pose2d(62, -13, Math.toRadians(180)));
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
