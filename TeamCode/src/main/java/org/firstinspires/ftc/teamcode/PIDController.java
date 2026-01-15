package org.firstinspires.ftc.teamcode;

public class PIDController {
    private double kP, kI, kD;
    private double integral = 0;
    private double lastError = 0;
    private long lastTime = 0;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double update(double error) {
        long now = System.nanoTime();
        double dt = (lastTime == 0) ? 0 : (now - lastTime) / 1e9;
        lastTime = now;

        if (dt > 0) {
            integral += error * dt;
            double derivative = (error - lastError) / dt;
            lastError = error;
            return kP * error + kI * integral + kD * derivative;
        }

        lastError = error;
        return kP * error;
    }

    public void reset() {
        integral = 0;
        lastError = 0;
        lastTime = 0;
    }
}