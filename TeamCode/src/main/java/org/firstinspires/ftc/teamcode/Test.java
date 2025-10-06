package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Test Drive", group="Linear OpMode")

public class Test extends LinearOpMode {

    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor flywheel;

    @Override
    public void runOpMode() {
        frontRight = hardwareMap.dcMotor.get("frontRight");
        frontLeft  = hardwareMap.dcMotor.get("frontLeft");
        backRight  = hardwareMap.dcMotor.get("backRight");
        backLeft   = hardwareMap.dcMotor.get("backLeft");
        flywheel   = hardwareMap.dcMotor.get("flywheel");

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        //flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            double x  = gamepad1.left_stick_x;
            double y  = -gamepad1.left_stick_y;
            double rx = -gamepad1.right_stick_x;

            if(gamepad1.x){
                flywheel.setPower(1.0);
            } else if (gamepad1.y){
                flywheel.setPower(-1);
            } else {
                flywheel.setPower(0);
            }

            double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rx),1);
            double frontRightPower = (y - x - rx) / denominator;
            double frontLeftPower  = (y + x + rx) / denominator;
            double backRightPower  = (y + x - rx) / denominator;
            double backLeftPower   = (y - x + rx) / denominator;
            frontRight.setPower(frontRightPower);
            frontLeft.setPower(frontLeftPower);
            backRight.setPower(backRightPower);
            backLeft.setPower(backLeftPower);

            telemetry.update();
        }
    }
}
