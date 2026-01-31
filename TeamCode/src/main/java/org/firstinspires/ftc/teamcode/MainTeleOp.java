package org.firstinspires.ftc.teamcode;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@TeleOp
@Configurable
public class MainTeleOp extends OpMode {
    private Follower follower;
    private boolean parking;
    private Supplier<PathChain> BlueParkingPathChain;
    private Supplier<PathChain> RedParkingPathChain;
    private TelemetryManager telemetryM;

    private final ElapsedTime runtime = new ElapsedTime();
    private Intake intake;


    private Lift lift;

    private BallLaunch ballLaunch;
    public static double ballVelocity = 2000;
    public ElapsedTime CurrentTimePassed;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(Globals.PoseX, Globals.PoseY, Globals.PoseHeading));
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        RedParkingPathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(31.9, 26.4))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(225), 0.8))
                .build();
        BlueParkingPathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(111.9, 26.4))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(315), 0.8))
                .build();

        lift = new Lift(hardwareMap);
        ballLaunch = new BallLaunch(hardwareMap, gamepad1);
        intake = new Intake(hardwareMap);
        CurrentTimePassed = new ElapsedTime();
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }
    @Override
    public void start() {
        runtime.reset();
        follower.startTeleopDrive();

        lift.setTargetTicks(Lift.minTicks);
    }
    @Override
    public void loop() {
        follower.update();

        if (gamepad2.dpad_up && !gamepad2.dpad_down) {
            lift.up();
        } else if (gamepad2.dpad_down && !gamepad2.dpad_up) {
            lift.down();
        }

        lift.update();

        if (gamepad1.dpad_up || gamepad2.x) {
            ballLaunch.forceLaunch = true;
            ballLaunch.setTargetVelocity(ballVelocity);
        } else {
            ballLaunch.forceLaunch = false;
        }

        ballLaunch.update();


        if (ballLaunch.currentState == BallLaunch.STATES.READY_TO_LAUNCH) {
            gamepad1.rumble(50);
        }

        if (gamepad1.a) {
            ballLaunch.launch();
        }

        double multiplier;
        if (gamepad1.left_bumper) {
            multiplier = 0.3;
        } else {
            multiplier = 1;
        }

        if (!parking) {
            if (gamepad1.left_trigger > 0.05 && gamepad1.right_trigger <= 0.05) { // Strafe left using left trigger
                follower.setTeleOpDrive(
                        0,
                        gamepad1.left_trigger / 1.5,
                        0,
                        true
                );

            } else if (gamepad1.right_trigger > 0.05 && gamepad1.left_trigger <= 0.05) { // Strafe right using right trigger
                follower.setTeleOpDrive(
                        0,
                        -gamepad1.right_trigger / 1.5,
                        0,
                        true
                );
            } else { // Both triggers are pressed, normal driving
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * multiplier,
                        -gamepad1.left_stick_x * multiplier * 0.1,
                        -gamepad1.right_stick_x * multiplier * 0.75,
                        true
                );
            }
        }

        if (CurrentTimePassed.seconds() > 160){
            gamepad1.rumble(2000);
            gamepad2.rumble(2000);
        }

        if (gamepad2.right_bumper && !gamepad2.left_bumper) {
            intake.pushOut();
        } else if (gamepad2.left_bumper && !gamepad2.right_bumper) {
            intake.pullIn();
        } else {
            intake.stop();
        }


//        if (gamepad1.yWasPressed()) {
//            if (Globals.isRed) {
//                follower.followPath(RedParkingPathChain.get());
//            } else {
//                follower.followPath(BlueParkingPathChain.get());
//            }
//            parking = true;
//        }

        if (gamepad1.rightBumperWasPressed()) {
            follower.turnTo(LaunchCalculator.heading(follower.getPose().getX(), follower.getPose().getY(), Globals.isRed));
        }
        if (gamepad1.rightBumperWasReleased()) {
            follower.startTeleopDrive();
        }

//
//        if (parking && (gamepad1.xWasPressed() || !follower.isBusy())) {
//            follower.startTeleopDrive();
//            parking = false;
//        }

        telemetryM.addLine("---- Robot Position ----");
        telemetryM.addData("X", follower.getPose().getX());
        telemetryM.addData("Y", follower.getPose().getY());
        telemetryM.addData("heading", follower.getPose().getHeading());
        telemetryM.addData("targetHeading", LaunchCalculator.heading(follower.getPose().getX(), follower.getPose().getY(), Globals.isRed));
        telemetryM.addLine("");
        telemetryM.addLine("---- Robot Status ----");
        telemetryM.debug("Ball Launch (state, target_vel, current_vel)", ballLaunch.currentState, ballLaunch.getTargetVelocity(), ballLaunch.getVelocity());
        telemetryM.debug("lift (target, left, right)", lift.getTargetTicks(), lift.getLeftTicks(), lift.getRightTicks());

        //telemetryM.debug("total heading:" + follower.getTotalHeading());
        telemetryM.update(telemetry);
    }
}