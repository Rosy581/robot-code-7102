package org.firstinspires.ftc.teamcode.hardware;

public class otos {
    SparkFunOTOS myOtos;
    SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, -8.5, 0);
    public void config(double x, double y){
        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        myOtos.setOffset(offset);

        myOtos.setLinearScalar(1.03125);
        myOtos.setAngularScalar(1.0);
       
        myOtos.calibrateImu();
        
        myOtos.resetTracking();

        Sparkfun.Pose2D startingPos = new SparkFunOTOS.Pose2D(x, y, 0);

        myOtos.setPosition(currentPosition);
    }
}