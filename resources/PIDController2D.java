package org.firstinspires.ftc.teamcode.resources;

public class PIDController2D {
    private double kpX, kiX, kdX; // Proportional, Integral, Derivative gains for X
    private double kpY, kiY, kdY; // Proportional, Integral, Derivative gains for Y
    private double kpRotation, kiRotation, kdRotation; // Proportional, Integral, Derivative gains for Rotation

    private double setpointX, setpointY, setpointRotation; // Desired target values for X, Y, and Rotation
    private double integralX, integralY, integralRotation; // Integral terms for X, Y, and Rotation
    private double lastErrorX, lastErrorY, lastErrorRotation; // Last error values for X, Y, and Rotation
    private long lastTime; // Last time the update was called

    public boolean atTarget;

    public PIDController2D(double kpX, double kiX, double kdX, double kpY, double kiY, double kdY,
                            double kpRotation, double kiRotation, double kdRotation) {
        this.kpX = kpX;
        this.kiX = kiX;
        this.kdX = kdX;
        this.kpY = kpY;
        this.kiY = kiY;
        this.kdY = kdY;
        this.kpRotation = kpRotation;
        this.kiRotation = kiRotation;
        this.kdRotation = kdRotation;
        this.integralX = 0;
        this.integralY = 0;
        this.integralRotation = 0;
        this.lastErrorX = 0;
        this.lastErrorY = 0;
        this.lastErrorRotation = 0;
        this.lastTime = System.currentTimeMillis();
        this.atTarget = false;
    }

    public void setSetpoints(double setpointX, double setpointY, double setpointRotation) {
        this.setpointX = setpointX;
        this.setpointY = setpointY;
        this.setpointRotation = setpointRotation;
    }

    public double[] calculate(double currentX, double currentY, double currentRotation) {
        long currentTime = System.currentTimeMillis();
        double dt = (currentTime - lastTime) / 1000.0; // Convert milliseconds to seconds

        // Calculate errors
        double errorX = setpointX - currentX;
        double errorY = setpointY - currentY;
        double errorRotation = setpointRotation - currentRotation;

        // Proportional terms
        double pTermX = kpX * errorX;
        double pTermY = kpY * errorY;
        double pTermRotation = kpRotation * errorRotation;

        // Integral terms
        integralX += errorX * dt;
        integralY += errorY * dt;
        integralRotation += errorRotation * dt;
        double iTermX = kiX * integralX;
        double iTermY = kiY * integralY;
        double iTermRotation = kiRotation * integralRotation;

        // Derivative terms
        double dTermX = 0;
        double dTermY = 0;
        double dTermRotation = 0;
        if (dt > 0) {
            dTermX = kdX * (errorX - lastErrorX) / dt;
            dTermY = kdY * (errorY - lastErrorY) / dt;
            dTermRotation = kdRotation * (errorRotation - lastErrorRotation) / dt;
        }

        // Save errors and time for next calculation
        lastErrorX = errorX;
        lastErrorY = errorY;
        lastErrorRotation = errorRotation;
        lastTime = currentTime;

        // Calculate total outputs
        double outputX = Math.abs(errorX) < 0.75 ? 0 : Math.min(pTermX + iTermX + dTermX, 1);
        double outputY = Math.abs(errorY) < 0.75 ? 0 : Math.min(pTermY + iTermY + dTermY, 1);
        double outputRotation = Math.abs(errorRotation) < 1 ? 0 : Math.min(pTermRotation + iTermRotation + dTermRotation, 1);

        outputX = errorX < 2.0 ? Math.min(0.25, outputX) : outputX;
        outputY = errorY < 2.0 ? Math.min(0.25, outputY) : outputY;

        atTarget = (outputX == 0 && outputY == 0 && outputRotation == 0);
        if(atTarget) reset();
        return new double[]{-outputX, -outputY, outputRotation}; // Return outputs for X, Y, and Rotation
    }

    public void reset() {
        integralX = 0;
        integralY = 0;
        integralRotation = 0;
        lastErrorX = 0;
        lastErrorY = 0;
        lastErrorRotation = 0;
        lastTime = System.currentTimeMillis();
    }
}