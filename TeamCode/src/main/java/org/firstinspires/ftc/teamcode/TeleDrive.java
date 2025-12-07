package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.GP;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.Robot.TEAMCOLOR;
import org.firstinspires.ftc.teamcode.configurables.Config.motorNames;


@TeleOp(name = "Main TeleOP", group = "Tele")

public class TeleDrive extends LinearOpMode {
    Robot robot;
    private double intakeSpeed = 0;
    private TEAMCOLOR teamColor = TEAMCOLOR.RED;

    @Override
    public void runOpMode() {
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        DcMotor frontRight = hardwareMap.dcMotor.get(motorNames.FrontRight);
        DcMotor frontLeft = hardwareMap.dcMotor.get(motorNames.FrontLeft);
        DcMotor backRight = hardwareMap.dcMotor.get(motorNames.BackRight);
        DcMotor backLeft = hardwareMap.dcMotor.get(motorNames.BackLeft);
        DcMotor intake = hardwareMap.dcMotor.get("Intake");

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        robot = new Robot(this, hardwareMap);
        waitForStart();
        gamepad1.runLedEffect(GP.RedLights);

        while (opModeIsActive()) {
            double x = gamepad1.left_stick_x;
            double y = - gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.squareWasPressed()) {
                intakeSpeed = (intakeSpeed == 0) ? 1 : 0;
                intake.setPower(intakeSpeed);
            }

            if (gamepad1.rightBumperWasPressed()) {
                robot.shoot();
            }

            if (gamepad1.psWasPressed()) {
                teamColor = (teamColor == TEAMCOLOR.RED) ? TEAMCOLOR.BLUE : TEAMCOLOR.RED;
                if (teamColor == TEAMCOLOR.RED) {
                    gamepad1.runLedEffect(GP.RedLights);
                } else {
                    gamepad1.runLedEffect(GP.BlueLights);
                }
            }

            double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rx), 1);
            double frontRightPower = (y - x - rx) / denominator;
            double frontLeftPower = (y + x + rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;

            frontRight.setPower(frontRightPower);
            frontLeft.setPower(frontLeftPower);
            backRight.setPower(backRightPower);
            backLeft.setPower(backLeftPower);
            robot.update();
            robot.aim(teamColor);
            telemetryM.addData("RPM", robot.RPM);
            telemetryM.addData("intake speed", intakeSpeed);
            telemetryM.addData("Team Color", (teamColor == TEAMCOLOR.RED) ? ("Red") : ("Blue"));
            telemetryM.addData("shooting", robot.shooting);
            telemetryM.addData("revved", robot.revved);
            telemetryM.addData("aiming", robot.aiming);
            telemetryM.addData("speed", robot.shooterPower);
            telemetryM.addData("target", Robot.target);
            telemetryM.addData("position", robot.turretMotor.getCurrentPosition());
            telemetryM.update(telemetry);
        }
    }
}
