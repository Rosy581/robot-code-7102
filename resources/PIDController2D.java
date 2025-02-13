package org.firstinspires.ftc.teamcode.resources;

public class PIDController2D {
    private double kpX, kiX, kdX; // Proportional, Integral, Derivative gains for X
    private double kpY, kiY, kdY; // Proportional, Integral, Derivative gains for Y
    private double kpR, kiR, kdR; // Proportional, Integral, Derivative gains for R

    private double setpointX, setpointY, setpointR; // Desired target values for X, Y, and R
    private double integralX, integralY, integralR; // Integral terms for X, Y, and R
    private double lastErrorX, lastErrorY, lastErrorR; // Last error values for X, Y, and R
    private long lastTime; // Last time the update was called

    public boolean atTarget;
    public double speedScale;

    public PIDController2D(double kpX, double kiX, double kdX, double kpY, double kiY, double kdY,
                           double kpR, double kiR, double kdR, double speedScale) {
        this.kpX = kpX;
        this.kiX = kiX;
        this.kdX = kdX;
        this.kpY = kpY;
        this.kiY = kiY;
        this.kdY = kdY;
        this.kpR = kpR;
        this.kiR = kiR;
        this.kdR = kdR;
        this.integralX = 0;
        this.integralY = 0;
        this.integralR = 0;
        this.lastErrorX = 0;
        this.lastErrorY = 0;
        this.lastErrorR = 0;
        this.lastTime = System.currentTimeMillis();
        this.atTarget = false;
        this.speedScale = Math.max(Math.abs(speedScale),1);
    }

    public void setTarget(double setpointX, double setpointY, double setpointR) {
        this.setpointX = setpointX;
        this.setpointY = setpointY;
        this.setpointR = setpointR;
        reset();
    }

    public double[] calculate(double currentX, double currentY, double currentR) {
        long currentTime = System.currentTimeMillis();
        double dt = (currentTime - lastTime) / 1000.0; // Convert milliseconds to seconds

        // Calculate errors
        double errorX = setpointX - currentX;
        double errorY = setpointY - currentY;
        double errorR = setpointR - currentR;

        // Proportional terms
        double pTermX = kpX * errorX;
        double pTermY = kpY * errorY;
        double pTermR = kpR * errorR;

        // Integral terms
        integralX += errorX * dt;
        integralY += errorY * dt;
        integralR += errorR * dt;
        double iTermX = kiX * integralX;
        double iTermY = kiY * integralY;
        double iTermR = kiR * integralR;

        // Derivative terms
        double dTermX = 0;
        double dTermY = 0;
        double dTermR = 0;
        if (dt > 0) {
            dTermX = kdX * (errorX - lastErrorX) / dt;
            dTermY = kdY * (errorY - lastErrorY) / dt;
            dTermR = kdR * (errorR - lastErrorR) / dt;
        }

        // Save errors and time for next calculation
        lastErrorX = errorX;
        lastErrorY = errorY;
        lastErrorR = errorR;
        lastTime = currentTime;

        // Calculate total outputs
        double outputX = Math.abs(errorX) < 0.75 ? 0 : Math.min(pTermX + iTermX + dTermX, 1);
        double outputY = Math.abs(errorY) < 0.75 ? 0 : Math.min(pTermY + iTermY + dTermY, 1);
        double outputR = Math.abs(errorR) < 1.0  ? 0 : Math.min(pTermR + iTermR + dTermR, 1);

        outputX = errorX < 2.0 ? Math.min(0.25, outputX) : outputX;
        outputY = errorY < 2.0 ? Math.min(0.25, outputY) : outputY;

        atTarget = (outputX == 0 && outputY == 0 && outputR == 0);
        if(atTarget) reset();
        return new double[]{-outputX / speedScale, -outputY / speedScale, outputR / speedScale};
    }

    public void setSpeedScale(double speed){
        speedScale = speed;
    }

    public void reset() {
        integralX = 0;
        integralY = 0;
        integralR = 0;
        lastErrorX = 0;
        lastErrorY = 0;
        lastErrorR = 0;
        lastTime = System.currentTimeMillis();
        atTarget = false;
    }
}