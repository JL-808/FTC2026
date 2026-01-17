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
        ballLaunch = new BallLaunch(hardwareMap);
        intake = new Intake(hardwareMap);
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
        telemetryM.update();

        if (gamepad2.dpad_up && !gamepad2.dpad_down) {
            lift.up();
        } else if (gamepad2.dpad_down && !gamepad2.dpad_up) {
            lift.down();
        }

        lift.update();

        if (gamepad1.dpadUpWasPressed()) {
            ballLaunch.forceLaunch = true;
            ballLaunch.setTargetVelocity(ballVelocity); // TODO: Adjust velocity as needed
        }
        if (gamepad1.dpadUpWasReleased()) {
            ballLaunch.forceLaunch = false;
        }

        ballLaunch.update();

        if (gamepad1.a) {
            ballLaunch.launch();
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
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        true
                );
            }
        }

        if (gamepad2.dpad_right) {
            intake.pushOut();
        } else if (gamepad2.dpad_left) {
            intake.pullIn();
        } else {
            intake.stop();
        }


        if (gamepad1.yWasPressed()) {
            if (Globals.isRed) {
                follower.followPath(RedParkingPathChain.get());
            } else {
                follower.followPath(BlueParkingPathChain.get());
            }
            parking = true;
        }

        if (parking && (gamepad1.xWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            parking = false;
        }


        telemetryM.debug("Ball Launch (State, target_vel, current_vel)", ballLaunch.currentState, ballLaunch.getTargetVelocity(), ballLaunch.getCurrentVelocity());
        telemetryM.debug("lift (target, left, right)", lift.getTargetTicks(), lift.getLeftTicks(), lift.getRightTicks());

        telemetryM.debug("x:" + follower.getPose().getX());
        telemetryM.debug("y:" + follower.getPose().getY());
        telemetryM.debug("heading:" + follower.getPose().getHeading());
        telemetryM.debug("total heading:" + follower.getTotalHeading());
        telemetryM.update(telemetry);
    }
}