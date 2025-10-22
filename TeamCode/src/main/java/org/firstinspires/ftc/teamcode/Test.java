package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.hardware.GP;
@TeleOp(name="Test Drive", group="Linear OpMode")

public class Test extends LinearOpMode {

    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor flywheel;
    private DcMotor shooting1;
    private DcMotor shooting2;
    private CRServo feedingServo;
    private Gamepad oldGp;

    @Override
    public void runOpMode() {
        frontRight   = hardwareMap.dcMotor.get("frontRight");
        frontLeft    = hardwareMap.dcMotor.get("frontLeft");
        backRight    = hardwareMap.dcMotor.get("backRight");
        backLeft     = hardwareMap.dcMotor.get("backLeft");
        flywheel     = hardwareMap.dcMotor.get("flywheel");
        shooting1    = hardwareMap.dcMotor.get("shooting1");
        shooting2    = hardwareMap.dcMotor.get("shooting2");
        feedingServo = hardwareMap.crservo.get("feedingServo");

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shooting2.setDirection(DcMotorSimple.Direction.REVERSE);

        //flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        double shootingPower = 0.5;

        waitForStart();
        oldGp = gamepad1;
        while (opModeIsActive()) {
            double x  = gamepad1.left_stick_x;
            double y  = -gamepad1.left_stick_y;
            double rx = -gamepad1.right_stick_x;


            if(gamepad1.x && !(gamepad1.y)){
                flywheel.setPower(1.0);
            } else if (gamepad1.y && !(gamepad1.x)){
                flywheel.setPower(-1);
            } else {
                flywheel.setPower(0);
            }

            if(gamepad1.a){
                shooting1.setPower(shootingPower);
                shooting2.setPower(shootingPower);
            } else {
                shooting1.setPower(0);
                shooting2.setPower(0);
            }

            telemetry.addData("ShootingPower",shootingPower);
            telemetry.addData("Up",gamepad1.dpad_up);
            telemetry.addData("LastUp",oldGp.dpad_up);
            telemetry.addData("Down",gamepad1.dpad_down);
            telemetry.addData("LastDown",oldGp.dpad_down);

            if(gamepad1.dpad_up){
                feedingServo.setPower(1.0);
            } else if (gamepad1.dpad_down){
                feedingServo.setPower(-1.0);
            } else {
                feedingServo.setPower(0);
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
            oldGp = gamepad1;
            telemetry.update();
        }
    }
}
