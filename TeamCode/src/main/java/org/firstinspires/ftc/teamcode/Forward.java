package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.configurables.Config;

@Autonomous

public class Forward extends LinearOpMode {

    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;

    @Override
    public void runOpMode() throws InterruptedException  {
        frontRight   = hardwareMap.dcMotor.get(Config.partNames.FrontRight);
        frontLeft	 = hardwareMap.dcMotor.get(Config.partNames.FrontLeft);
        backRight  	 = hardwareMap.dcMotor.get(Config.partNames.BackRight);
        backLeft 	 = hardwareMap.dcMotor.get(Config.partNames.BackLeft);

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {
            frontRight.setPower(1);
            frontLeft.setPower(1);
            backRight.setPower(1);
            backLeft.setPower(1);
            Thread.sleep(3000);
            frontRight.setPower(0);
            frontLeft.setPower(0);
            backRight.setPower(0);
            backLeft.setPower(0);
        }
    }
}
