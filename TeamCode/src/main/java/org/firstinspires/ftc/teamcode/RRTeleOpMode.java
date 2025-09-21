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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name="Main OpMode", group="Main")
public class RRTeleOpMode extends OpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor intake = null;

    private MecanumDrive drive = null;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        intake = hardwareMap.get(DcMotor.class, "intake");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        TelemetryPacket packet = new TelemetryPacket();

        // updated based on gamepads

//        // update running actions
//        List<Action> newActions = new ArrayList<>();
//        for (Action action : runningActions) {
//            action.preview(packet.fieldOverlay());
//            if (action.run(packet)) {
//                newActions.add(action);
//            }
//        }
//        runningActions = newActions;

        dash.sendTelemetryPacket(packet);

        PoseVelocity2d drivePower = new PoseVelocity2d(
                new Vector2d (
                        gamepad1.left_stick_x,
                        -gamepad1.left_stick_y
                ),
                gamepad1.right_stick_x
        );

        drive.setDrivePowers(drivePower);

        // Update Road Runner’s localization + drive system
        drive.updatePoseEstimate();

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}