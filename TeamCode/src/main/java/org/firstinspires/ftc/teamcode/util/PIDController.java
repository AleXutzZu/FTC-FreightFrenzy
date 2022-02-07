package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController implements Driveable {
    private final double P, I, D, target;
    private double accumulatedError = 0;
    private final ElapsedTime time = new ElapsedTime();
    private double lastError = 0f;
    private double lastTime = 0f;

    public PIDController(double p, double i, double d, double target) {
        P = p;
        I = i;
        D = d;
        this.target = target;
    }

    @Override
    public double update(double yaw) {
        double error = target - yaw;
        // P coefficient
        error %= 360;
        error += 360;
        error %= 360;
        if (error > 180) error -= 360;

        //I coefficient
        accumulatedError += error;

        if (Math.abs(error) < 1) accumulatedError = 0f;
        accumulatedError = Math.abs(accumulatedError) * Math.signum(error);

        //D coefficient
        double slope = 0;
        if (lastTime > 0) {
            slope = (error - lastError) / ( time.milliseconds() - lastTime);
        }
        lastTime =  time.milliseconds();
        lastError = error;

        //Motor Power calculation
        return 0.1 * Math.signum(error) + 0.9 * Math.tanh(P * error + I * accumulatedError + D * slope);
    }
}
