package org.firstinspires.ftc.teamcode.resources;

public class PIDController2D {
    private double kpX, kiX, kdX; // Proportional, Integral, Derivative gains for X
    private double kpY, kiY, kdY; // Proportional, Integral, Derivative gains for Y

    private double setpointX, setpointY; // Desired target values for X and Y
    private double integralX, integralY; // Integral terms for X and Y
    private double lastErrorX, lastErrorY; // Last error values for X and Y
    private long lastTime; // Last time the update was called

    public PIDController2D(double kpX, double kiX, double kdX, double kpY, double kiY, double kdY) {
        this.kpX = kpX;
        this.kiX = kiX;
        this.kdX = kdX;
        this.kiY = kiY;
        this.kdY = kdY;
        this.integralX = 0;
        this.integralY = 0;
        this.lastErrorX = 0;
        this.lastErrorY = 0;
        this.lastTime = System.currentTimeMillis();
    }

    public void setSetpoints(double setpointX, double setpointY) {
        this.setpointX = setpointX;
        this.setpointY = setpointY;
    }

    public double[] calculate(double currentX, double currentY) {
        long currentTime = System.currentTimeMillis();
        double dt = (currentTime - lastTime) / 1000.0; // Convert milliseconds to seconds

        // Calculate errors
        double errorX = setpointX - currentX;
        double errorY = setpointY - currentY;

        // Proportional terms
        double pTermX = kpX * errorX;
        double pTermY = kpY * errorY;

        // Integral terms
        integralX += errorX * dt;
        integralY += errorY * dt;
        double iTermX = kiX * integralX;
        double iTermY = kiY * integralY;

        // Derivative terms
        double dTermX = 0;
        double dTermY = 0;
        if (dt > 0) {
            dTermX = kdX * (errorX - lastErrorX) / dt;
            dTermY = kdY * (errorY - lastErrorY) / dt;
        }

        // Save errors and time for next calculation
        lastErrorX = errorX;
        lastErrorY = errorY;
        lastTime = currentTime;

        // Calculate total outputs
        double outputX = pTermX + iTermX + dTermX;
        double outputY = pTermY + iTermY + dTermY;
        System.out.print("test");
        return new double[]{ -outputX, -outputY}; // Return both outputs
    }

    public void reset() {
        integralX = 0;
        integralY = 0;
        lastErrorX = 0;
        lastErrorY = 0;
        lastTime = System.currentTimeMillis();
    }
}
