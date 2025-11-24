package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.configurables.Config.shooterConstants;
@TeleOp(name="Main TeleOP", group="Tele")

public class TeleDrive extends LinearOpMode {
    private Gamepad prevGamepad = new Gamepad();
    private double intakeSpeed = 0;
    public double RPM = 0;
    public double shooterPower = 0;
    private double prevPos;
    private double lastTime;
    private DcMotor shooter;
    public boolean shooting = false;
    public boolean onTarget = true;

    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor frontLeft  = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backRight  = hardwareMap.dcMotor.get("backRight");
        DcMotor backLeft   = hardwareMap.dcMotor.get("backLeft");

        DcMotor intake     = hardwareMap.dcMotor.get("flywheel");
        shooter            = hardwareMap.dcMotor.get("shooter");

        CRServo feedingServo = hardwareMap.crservo.get("feedingServo");

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //intake.setDirection(DcMotorSimple.Direction.REVERSE);
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

            if (gamepad1.x && !prevGamepad.x) {
                intakeSpeed = (intakeSpeed == 0)?1:0;
                intake.setPower(intakeSpeed);
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

            telemetryM.addData("intake speed",intakeSpeed);

            prevGamepad.copy(gamepad1);

            telemetryM.update(telemetry);
        }
    }
    public double getRPM(){
        RPM = Math.abs((shooter.getCurrentPosition())-prevPos)/1680*(1000/(lastTime-getRuntime()));
        prevPos = shooter.getCurrentPosition();
        lastTime = getRuntime();
        return RPM;
    }
    private void shoot(){
        if(shooting){
            if((Math.abs(shooterConstants.targetRPM - RPM) > shooterConstants.tolerance)) {
                shooterPower = shooterConstants.Kp * (shooterConstants.targetRPM - RPM) + shooterConstants.Kv * (shooterConstants.targetRPM);
                shooter.setPower(shooterPower);
            } else {
                //move ball into thing
                shooting = false;
            }
        }
    }
    private void target(){
        if(!onTarget){
            /*
            idrk how im gonna do this would rather not just blindly search because that'd be super slow
             */
        }
    }

}
