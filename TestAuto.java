package org.firstinspires.ftc.teamcode;

import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous

public class TestAuto extends LinearOpMode {
    IMU imu;
    @Override
    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        
        return;
    }
}