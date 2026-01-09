package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.Robot.TEAMCOLOR;
import org.firstinspires.ftc.teamcode.configurables.Config.partNames;

import java.util.concurrent.TimeUnit;


@TeleOp(name = "Main TeleOP", group = "Tele")

public class TeleDrive extends LinearOpMode {
    Robot robot;
    private double intakeSpeed = 0;
    private double feederPower = 0;
    private TEAMCOLOR teamColor = TEAMCOLOR.RED;
    private Deadline pushTime = new Deadline(500, TimeUnit.MILLISECONDS);
    @Override
    public void runOpMode() {
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        DcMotor frontRight = hardwareMap.dcMotor.get(partNames.FrontRight);
        DcMotor frontLeft = hardwareMap.dcMotor.get(partNames.FrontLeft);
        DcMotor backRight = hardwareMap.dcMotor.get(partNames.BackRight);
        DcMotor backLeft = hardwareMap.dcMotor.get(partNames.BackLeft);
        DcMotor intake = hardwareMap.dcMotor.get("Intake");

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        robot = new Robot(this, hardwareMap);
        waitForStart();
//        gamepad1.runLedEffect(GP.RedLights);

        while (opModeIsActive()) {
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.squareWasPressed()) {
                intakeSpeed = (intakeSpeed == 1 ? 0 : 1);
                intake.setPower(intakeSpeed);
            }

            if (gamepad1.rightBumperWasPressed()) {
                robot.shoot();
                intakeSpeed = 0;
                intake.setPower(intakeSpeed);
            }

            if (gamepad1.right_trigger > 0.1){
                robot.push();
            }

            if(gamepad2.crossWasPressed()){
                robot.turnToAngle(0);
            }

            if(gamepad1.triangleWasPressed()){
                feederPower = (feederPower == 1 ? 0 : 1);
                robot.feeder.setPower(feederPower);
            }

            /*
            if (gamepad1.psWasPressed()) {
                teamColor = (teamColor == TEAMCOLOR.RED) ? TEAMCOLOR.BLUE : TEAMCOLOR.RED;
                if (teamColor == TEAMCOLOR.RED) {
                    gamepad1.runLedEffect(GP.RedLights);
                } else {
                    gamepad1.runLedEffect(GP.BlueLights);
                }
            }
            */


            double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rx), 1);
            double frontRightPower = (y - x - rx) / denominator;
            double frontLeftPower = (y + x + rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;

            frontRight.setPower(frontRightPower);
            frontLeft.setPower(frontLeftPower);
            backRight.setPower(backRightPower);
            backLeft.setPower(backLeftPower);

            robot.update(teamColor);
            telemetryM.addData("RPM", robot.RPM);
            telemetryM.addData("intake speed", intakeSpeed);
            telemetryM.addData("Team Color", (teamColor == TEAMCOLOR.RED) ? ("Red") : ("Blue"));
            telemetryM.addData("shooting", robot.shooting);
            telemetryM.addData("revved", robot.revved);
            telemetryM.addData("aiming", robot.aiming);
            telemetryM.addData("speed", robot.shooterPower);
            telemetryM.addData("target", Robot.target);
            telemetryM.addData("position", robot.turretMotor.getCurrentPosition());
            telemetryM.addData("TAG POSITION",robot.point);
            telemetryM.addData("odo",robot.odo.getHeading(AngleUnit.DEGREES));
            telemetryM.addData("f",robot.getTurretRotation());
            telemetryM.update(telemetry);
        }
    }
    void updateIntake(DcMotor intake){
        intakeSpeed = (intakeSpeed == 1 ? 0 : 1);
        intake.setPower(intakeSpeed);
    }
    void updateIntake(DcMotor intake,double speed){
        intake.setPower(speed);
    }
}
