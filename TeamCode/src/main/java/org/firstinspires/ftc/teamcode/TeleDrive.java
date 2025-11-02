package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="Main TeleOP", group="Tele")

public class TeleDrive extends LinearOpMode {
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor flywheel;
    private DcMotor shootingLeft;
    private DcMotor shootingRight;
    private CRServo feedingServo;
    private double intakeSpeed = 0;
    private double leftRPM      = 0;
    private double rightRPM     = 0;
    private double leftPrevPos  = 0;
    private double rightPrevPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        frontRight   = hardwareMap.dcMotor.get("frontRight");
        frontLeft	 = hardwareMap.dcMotor.get("frontLeft");
        backRight	 = hardwareMap.dcMotor.get("backRight");
        backLeft	 = hardwareMap.dcMotor.get("backLeft");
        flywheel	 = hardwareMap.dcMotor.get("flywheel");
        shootingLeft	 = hardwareMap.dcMotor.get("shooting1");
        shootingRight  	 = hardwareMap.dcMotor.get("shooting2");
        feedingServo = hardwareMap.crservo.get("feedingServo");

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shootingRight.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        double rightShootingPower = 0.45;
        double leftShootingPower  = 0.45;
        waitForStart();

        while (opModeIsActive()) {
            double x  =  gamepad1.left_stick_x;
            double y  = -gamepad1.left_stick_y;
            double rx =  gamepad1.right_stick_x;


            leftRPM  = (leftPrevPos  - shootingLeft.getCurrentPosition())/28;
            rightRPM = (rightPrevPos - shootingRight.getCurrentPosition())/28;

            if((leftShootingPower > 0.0) && (rightShootingPower > 0.0)){
                if((rightRPM+0.005)>= 0.21){
                    rightShootingPower -= 0.01;
                } else if ((rightRPM-0.005)<=0.21){
                    rightShootingPower += 0.01;
                }
                if((leftRPM+0.005)>= 0.21){
                    leftShootingPower -= 0.01;
                } else if ((leftRPM-0.005)<=0.21){
                    leftShootingPower = leftShootingPower + 0.01;
                }
            }


            if(gamepad1.left_bumper){
                feedingServo.setPower(1.0);
            } else if (gamepad1.left_trigger > 0.1){
                feedingServo.setPower(-1.0);
            } else {
                feedingServo.setPower(0.0);
            }

            if(gamepad1.y){
                shootingLeft.setPower(leftShootingPower);
                shootingRight.setPower(rightShootingPower);
            }
            if(gamepad1.b){
                shootingLeft.setPower(0);
                shootingRight.setPower(0);
            }

            if(gamepad1.x){
                flywheel.setPower(1);
            }

            if(gamepad1.a) {
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


            telemetry.addData("leftShootingPower", leftShootingPower);
            telemetry.addData("righttShootingPower",rightShootingPower);
            telemetry.addData("intake speed",intakeSpeed);
            telemetry.addData("LeftSpeed",leftRPM);
            telemetry.addData("RightSpeed",rightRPM);

            leftPrevPos  = shootingLeft.getCurrentPosition();
            rightPrevPos = shootingRight.getCurrentPosition();

            telemetry.update();
        }
    }
}
