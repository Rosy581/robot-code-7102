package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Otos {
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

        SparkFunOTOS.Pose2D startingPos = new SparkFunOTOS.Pose2D(x, y, 0);

        myOtos.setPosition(startingPos);
    }
}