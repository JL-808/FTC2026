// Nichola Chen 2025-2026

package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Configurable
public class BallLaunch {
    public enum STATES {
        IDLE,
        SPINNING_UP, // outtake is spinning up to velocity
        READY_TO_LAUNCH, // outtake is at velocity and ready to launch using launch()
        LAUNCHING, // currently launching a ball (busy)
        SERVO_DOWN, // servo is returning to start position
        RELOADING, // waiting for reload time before next launch
    }

    public STATES currentState = STATES.IDLE;

    public int launchCount = 0;
    public boolean forceLaunch = false; // If true, prepares launch even if launchCount is 0
    private double velocity = 0;

    public static int reloadTime = 200; // minimum time (ms) between launches
    public static int servoTime = 200; // time (ms) for servo to move to launch position

    public static double servoStartPosition = 0.83;
    public static double servoLaunchPosition = 0.5;

    public static double velocityTolerance = 20;

    private final Timer launchTimer = new Timer();
    private final DcMotorEx outtake;
    private final VoltageSensor batteryVoltageSensor;
    private final Servo launchServo;

    public static double P = 0.015;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0.000495;

    private final PIDController PID = new PIDController(P, I, D);

    private double v;

    public BallLaunch(HardwareMap hardwareMap) {
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake.setDirection(DcMotor.Direction.REVERSE);

        launchServo = hardwareMap.get(Servo.class, "launch");
        launchServo.setDirection(Servo.Direction.REVERSE);

        launchServo.setPosition(servoStartPosition);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public void update() {
        v =  outtake.getVelocity();
        switch (currentState) {
            case IDLE:
                if (launchCount > 0 || forceLaunch) {
                    currentState = STATES.SPINNING_UP;
                    reset();
                }
                break;
            case SPINNING_UP:
                motor_update();

                if (launchCount == 0 && !forceLaunch) {
                    currentState = STATES.IDLE;
                    outtake.setPower(0);
                    break;
                }

                if (Math.abs(v - velocity) <= velocityTolerance) {
                    currentState = STATES.READY_TO_LAUNCH;
                }
                break;
            case READY_TO_LAUNCH:
                motor_update();

                if (launchCount == 0 && !forceLaunch) {
                    currentState = STATES.IDLE;
                    outtake.setPower(0);
                    break;
                }
                if (Math.abs(v - velocity) > velocityTolerance) {
                    currentState = STATES.SPINNING_UP;
                    break;
                }
                break;
            case LAUNCHING:
                motor_update();

                if (launchTimer.getElapsedTime() > servoTime) {
                    currentState = STATES.SERVO_DOWN;
                    launchServo.setPosition(servoStartPosition);
                    launchTimer.resetTimer();
                }
                break;
            case SERVO_DOWN:
                if (launchCount > 0 || forceLaunch) {
                    motor_update();
                }

                if (launchTimer.getElapsedTime() > servoTime) {
                    if (launchCount > 0 || forceLaunch) {
                        currentState = STATES.RELOADING;

                        launchTimer.resetTimer();
                    } else {
                        currentState = STATES.IDLE;
                        outtake.setPower(0);
                    }
                }
                break;
            case RELOADING:
                motor_update();

                if (launchCount == 0 && !forceLaunch) {
                    currentState = STATES.IDLE;
                    outtake.setPower(0);

                    break;
                }

                if (launchTimer.getElapsedTime() > reloadTime) {
                    if (launchCount > 0 || forceLaunch) {
                        if (Math.abs(v - velocity) <= velocityTolerance) {
                            currentState = STATES.READY_TO_LAUNCH;

                        } else {
                            currentState = STATES.SPINNING_UP;
                        }
                    } else {
                        currentState = STATES.IDLE;
                        outtake.setPower(0);
                    }
                }
                break;
        }
    }
    public void reset() {
        PID.reset();
    }

    public boolean launch() {
        // Returns true if launch initiated

        if (currentState == STATES.READY_TO_LAUNCH && (forceLaunch || launchCount > 0)) {
            currentState = STATES.LAUNCHING;
            launchServo.setPosition(servoLaunchPosition);
            launchTimer.resetTimer();

            if (launchCount > 0) {
                launchCount--;
            }

            return true;
        }
        return false;
    }

    private void motor_update() {
        double e = PID.update(velocity - v) + F * (12/batteryVoltageSensor.getVoltage()) * velocity;
        if (e > 1) e = 1;
        if (e < -1) e = -1;
        outtake.setPower(-e);
    }

    public double getVelocity() {
        return v;
    }
    public void setTargetVelocity(double targetVelocity) {
        if (velocity != targetVelocity) {
            reset();
        }
        velocity = targetVelocity;

        if (currentState == STATES.READY_TO_LAUNCH) {
            if (Math.abs(v - velocity) > velocityTolerance) {
                currentState = STATES.SPINNING_UP;
            }
        }
    }

    public double getTargetVelocity() {
        return velocity;
    }
}
