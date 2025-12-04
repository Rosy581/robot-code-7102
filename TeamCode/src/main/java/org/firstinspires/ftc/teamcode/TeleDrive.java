package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.hardware.GP;
import org.firstinspires.ftc.teamcode.hardware.Turret;
import org.firstinspires.ftc.teamcode.hardware.Turret.TEAMCOLOR;

@TeleOp(name="Main TeleOP", group="Tele")

public class TeleDrive extends LinearOpMode {
    Turret turret;
    private double intakeSpeed = 0;
    private TEAMCOLOR teamColor = TEAMCOLOR.RED;

    @Override
    public void runOpMode() {
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        turret             = new Turret(this, hardwareMap,"Turret","AimServo","Shooter","Camera");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor frontLeft  = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backRight  = hardwareMap.dcMotor.get("backRight");
        DcMotor backLeft   = hardwareMap.dcMotor.get("backLeft");
        DcMotor intake     = hardwareMap.dcMotor.get("intake");

        CRServo feedingServo = hardwareMap.crservo.get("feedingServo");

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //intake.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            double x  =  gamepad1.left_stick_x;
            double y  = -gamepad1.left_stick_y;
            double rx =  gamepad1.right_stick_x;

            if(gamepad1.leftBumperWasPressed()){
                feedingServo.setPower(1.0);
            } else if (gamepad1.left_trigger > 0.1){
                feedingServo.setPower(-1.0);
            } else {
                feedingServo.setPower(0.0);
            }

            if (gamepad1.squareWasPressed()) {
                intakeSpeed = (intakeSpeed == 0)?1:0;
                intake.setPower(intakeSpeed);
            }

            if(gamepad1.rightBumperWasPressed()){
                turret.shoot(teamColor);
            }

            if (gamepad1.psWasPressed()){
                teamColor = (teamColor == TEAMCOLOR.RED)?TEAMCOLOR.BLUE:TEAMCOLOR.RED;
                if(teamColor == TEAMCOLOR.RED){
                    gamepad1.runLedEffect(GP.RedLights);
                } else {
                    gamepad1.runLedEffect(GP.BlueLights);
                }
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
            turret.updateRPM();
            turret.aim(teamColor);
            telemetryM.addData("RPM",turret.RPM);
            telemetryM.addData("intake speed",intakeSpeed);
            telemetryM.addData("Team Color",(teamColor == TEAMCOLOR.RED)?("Red"):("Blue"));
            telemetryM.update(telemetry);
        }
    }
}
