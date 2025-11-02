package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name="Main TeleOP", group="Tele")

public class TeleDrive extends LinearOpMode {
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor flywheel;
    private DcMotor shooting1;
    private DcMotor shooting2;
    private CRServo feedingServo;
    private double intakeSpeed = 0;
    private boolean oldX = false;
    private boolean oldY = false;
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
        shooting1	 = hardwareMap.dcMotor.get("shooting1");
        shooting2  	 = hardwareMap.dcMotor.get("shooting2");
        feedingServo = hardwareMap.crservo.get("feedingServo");

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shooting2.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        double shootingPower = 0.45;
        waitForStart();

        while (opModeIsActive()) {
            double x  =  gamepad1.left_stick_x;
            double y  = -gamepad1.left_stick_y;
            double rx =  gamepad1.right_stick_x;

            if(gamepad1.left_bumper){
                feedingServo.setPower(1.0);
            } else if (gamepad1.left_trigger > 0.1){
                feedingServo.setPower(-1.0);
            } else {
                feedingServo.setPower(0.0);
            }

            if(gamepad1.y){
                shooting1.setPower(shootingPower);
                shooting2.setPower(shootingPower);
            }
            if(gamepad1.b){
                shooting1.setPower(0);
                shooting2.setPower(0);
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

            leftRPM  = (leftPrevPos  - shooting1.getCurrentPosition())/28;
            rightRPM = (rightPrevPos - shooting2.getCurrentPosition())/28;

            frontRight.setPower(frontRightPower);
            frontLeft.setPower(frontLeftPower);
            backRight.setPower(backRightPower);
            backLeft.setPower(backLeftPower);


            telemetry.addData("ShootingPower",shootingPower);
            telemetry.addData("intake speed",intakeSpeed);
            telemetry.addData("LeftSpeed",leftRPM);
            telemetry.addData("RightSpeed",rightRPM);

            oldX = gamepad1.x;
            oldY = gamepad1.y;

            leftPrevPos  = shooting1.getCurrentPosition();
            rightPrevPos = shooting2.getCurrentPosition();

            telemetry.update();
        }
    }
}
