package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Autonomous (name="Red Top", group="Red")
public class RedTopAuto extends OpMode {
    private final Pose startPose = new Pose(144-26.5, 130.6, Math.toRadians(-323.7));
    private final Pose launchPose = new Pose(83, 84, LaunchCalculator.heading(83, 84, true));
    public enum STATES {
        INIT,
        TO_INITIAL_LAUNCH,
        INITIAL_LAUNCH,
        TO_INTAKE_1,
        INTAKE_1,
        EXIT_INTAKE_1,
        TO_LAUNCH_1,
        LAUNCH_1,
        TO_INTAKE_2,
        INTAKE_2,
        TO_LAUNCH_2,
        LAUNCH_2,
        END

    }

    private STATES currentState = STATES.INIT;
    private Follower follower;

    private Lift lift;
    private BallLaunch ballLaunch;
    private Intake intake;

    private PathChain ToInitialLaunch;
    private PathChain ToIntake1;
    private PathChain Intake1;
    private PathChain ExitIntake1;
    private PathChain ToLaunch1;
    private PathChain ToIntake2;
    private PathChain Intake2;
    private PathChain ToLaunch2;
    private Supplier<PathChain> EndPathChain;
    private Timer opmodeTimer;

    public void buildPaths() {
        ToInitialLaunch = follower.pathBuilder().addPath(
                        new BezierLine(
                                startPose,
                                launchPose
                        )
                ).setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading())
                .build();

        ToIntake1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                launchPose,
                                new Pose(launchPose.getX(), Globals.THIRD_ROW_ARTIFACTS),
                                new Pose(144 - Globals.BEGIN_INTAKE, Globals.THIRD_ROW_ARTIFACTS)
                        )
                ).setLinearHeadingInterpolation(launchPose.getHeading(), Math.toRadians(0))
                .setNoDeceleration()
                .build();

        Intake1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(144 - Globals.BEGIN_INTAKE, Globals.THIRD_ROW_ARTIFACTS),
                                new Pose(144 - Globals.THIRD_ROW_STOP_INTAKE, Globals.THIRD_ROW_ARTIFACTS)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        ExitIntake1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(144 - Globals.THIRD_ROW_STOP_INTAKE, Globals.THIRD_ROW_ARTIFACTS).getPose(),
                                new Pose(144 - Globals.BEGIN_INTAKE, Globals.THIRD_ROW_ARTIFACTS)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))
                .build(
        );

        ToLaunch1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(144 - Globals.BEGIN_INTAKE, Globals.THIRD_ROW_ARTIFACTS),
                                new Pose(launchPose.getX(), Globals.THIRD_ROW_ARTIFACTS),
                                launchPose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), launchPose.getHeading())
                .build();

        ToIntake2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                launchPose,
                                new Pose(launchPose.getX(), Globals.SECOND_ROW_ARTIFACTS),
                                new Pose(144 - Globals.BEGIN_INTAKE, Globals.SECOND_ROW_ARTIFACTS)
                        )
                ).setLinearHeadingInterpolation(launchPose.getHeading(), Math.toRadians(0))
                .build();

        Intake2 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(144 - Globals.BEGIN_INTAKE, Globals.SECOND_ROW_ARTIFACTS),
                        new Pose(144 - Globals.BEGIN_INTAKE, Globals.SECOND_ROW_ARTIFACTS)
                )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        ToLaunch2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(144 - Globals.SECOND_ROW_STOP_INTAKE, Globals.SECOND_ROW_ARTIFACTS),
                                new Pose(launchPose.getX(), Globals.SECOND_ROW_ARTIFACTS),
                                launchPose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), launchPose.getHeading())
                .build();

        EndPathChain = () -> follower.pathBuilder().addPath(
                        new Path(new BezierLine(
                                follower.getPose(),
                                new Pose(96, 72.000)
                        ))
                ).setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(0))
                .build();
    }

    @Override
    public void init() {
        Globals.isRed = true;

        opmodeTimer = new Timer();

        ballLaunch = new BallLaunch(hardwareMap);
        intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();

        lift.setTargetTicks(Lift.minTicks);
    }

    private void update() {
        switch (currentState) {
            case INIT:
                follower.followPath(ToInitialLaunch);
                currentState = STATES.TO_INITIAL_LAUNCH;

                ballLaunch.setTargetVelocity(Globals.SHORT_LAUNCH_VELOCITY);
                ballLaunch.launchCount = 3;

                break;
            case TO_INITIAL_LAUNCH:
                if (!follower.isBusy()) {
                    currentState = STATES.INITIAL_LAUNCH;
                }
                break;
            case INITIAL_LAUNCH:
                if (ballLaunch.currentState == BallLaunch.STATES.IDLE) {
                    follower.followPath(ToIntake1);
                    currentState = STATES.TO_INTAKE_1;
                } else {
                    if (ballLaunch.currentState == BallLaunch.STATES.READY_TO_LAUNCH_WAITED) {
                        ballLaunch.launch();
                    }
                }
                if (ballLaunch.currentState == BallLaunch.STATES.LAUNCHING) {
                    intake.pullIn();
                } else if (ballLaunch.currentState == BallLaunch.STATES.SERVO_DOWN) {
                    intake.stop();
                }
                break;
            case TO_INTAKE_1:
                if (!follower.isBusy()) {
                    follower.followPath(Intake1, Globals.INTAKE_DRIVE_POWER, true);
                    currentState = STATES.INTAKE_1;
                    intake.pullIn();
                }
                break;
            case INTAKE_1:
                if (!follower.isBusy()) {
                    follower.followPath(ExitIntake1);
                    currentState = STATES.EXIT_INTAKE_1;

                    ballLaunch.setTargetVelocity(Globals.SHORT_LAUNCH_VELOCITY);
                    ballLaunch.launchCount = 3;
                }
                break;
            case EXIT_INTAKE_1:
                if (!follower.isBusy()) {
                    intake.stop();
                    follower.followPath(ToLaunch1);
                    currentState = STATES.TO_LAUNCH_1;
                }
                break;
            case TO_LAUNCH_1:
                if (!follower.isBusy()) {
                    currentState = STATES.LAUNCH_1;
                }
                break;
            case LAUNCH_1:
                if (ballLaunch.currentState == BallLaunch.STATES.IDLE) {
                    currentState = STATES.TO_INTAKE_2;
                    follower.followPath(ToIntake2);
                } else {
                    if (ballLaunch.currentState == BallLaunch.STATES.READY_TO_LAUNCH_WAITED) {
                        ballLaunch.launch();
                    }
                }
                if (ballLaunch.currentState == BallLaunch.STATES.LAUNCHING) {
                    intake.pullIn();
                } else if (ballLaunch.currentState == BallLaunch.STATES.SERVO_DOWN) {
                    intake.stop();
                }
                break;
            case TO_INTAKE_2:
                if (!follower.isBusy()) {
                    follower.followPath(Intake2, Globals.INTAKE_DRIVE_POWER, true);
                    currentState = STATES.INTAKE_2;
                    intake.pullIn();
                }
                break;
            case INTAKE_2:
                if (!follower.isBusy()) {
                    intake.stop();
                    follower.followPath(ToLaunch2);
                    currentState = STATES.TO_LAUNCH_2;

                    ballLaunch.setTargetVelocity(Globals.SHORT_LAUNCH_VELOCITY);
                    ballLaunch.launchCount = 3;
                }
                break;
            case TO_LAUNCH_2:
                if (!follower.isBusy()) {
                    currentState = STATES.LAUNCH_2;
                }
                break;
            case LAUNCH_2:
                if (ballLaunch.currentState == BallLaunch.STATES.IDLE) {
                    currentState = STATES.END;
                    follower.followPath(EndPathChain.get());
                } else {
                    ballLaunch.launch();
                }
                if (ballLaunch.currentState == BallLaunch.STATES.LAUNCHING) {
                    intake.pullIn();
                } else if (ballLaunch.currentState == BallLaunch.STATES.SERVO_DOWN) {
                    intake.stop();
                }
                break;
        }
    }

    @Override
    public void loop() {
        follower.update();
        if (opmodeTimer.getElapsedTime() >= Globals.END_TIME * 1000 && currentState != STATES.END) {
            currentState = STATES.END;
            intake.stop();
            follower.followPath(EndPathChain.get());
        }

        update();

        ballLaunch.update();
        lift.update();


        telemetry.addData("ball launch", ballLaunch.currentState);
        telemetry.addData("ball launch velocity", ballLaunch.getVelocity());
        telemetry.addData("launch count", ballLaunch.launchCount);
        telemetry.addData("STATE", currentState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

    }

    @Override
    public void stop() {
        Globals.PoseX = follower.getPose().getX();
        Globals.PoseY = follower.getPose().getY();
        Globals.PoseHeading = follower.getPose().getHeading();
    }
}
