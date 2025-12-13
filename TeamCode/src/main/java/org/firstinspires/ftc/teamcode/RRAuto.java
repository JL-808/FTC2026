package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name="Test Roadrunner Auto")
public class RRAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        DcMotor outtake = hardwareMap.get(DcMotor.class, "outtake");

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0,0,0))
                        .lineToX(40.0)
                        .stopAndAdd(new MotorAction(outtake, 0.1))
                        .waitSeconds(3)
                        .stopAndAdd(new MotorAction(outtake, 0.25))
                        .lineToX(0)
                        .stopAndAdd(new MotorAction(outtake, .75))
                        .waitSeconds(.5)
                        .stopAndAdd(new MotorAction(outtake, 0))
                        .splineTo(new Vector2d(20,20), Math.PI/2)
                        .splineToConstantHeading(new Vector2d(0,0), Math.PI/2)
                        .build());

    }

    public class MotorAction implements Action {
        DcMotor motor;
        double power;


        public MotorAction (DcMotor s, double p)
        {
            this.motor = s;
            this.power = p;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            motor.setPower(power);
            return false;
        }
    }

    public class PatientMotorAction implements Action {
        DcMotor motor;
        double power;
        ElapsedTime timer;
        double time;


        public PatientMotorAction (DcMotor s, double p, double t)
        {
            this.motor = s;
            this.power = p;
            this.time = t;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null)
            {
                timer = new ElapsedTime();
                motor.setPower(power);
            }
            return (timer.seconds() < time);
        }
    }
}