package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.GP;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.Robot.TEAMCOLOR;


@TeleOp(name = "Main TeleOP", group = "Tele")

public class TeleDrive extends LinearOpMode {
    Robot robot;
    private double intakeSpeed = 0;
    private double feederPower = 0;
    private TEAMCOLOR teamColor = TEAMCOLOR.RED;

    @Override
    public void runOpMode() {
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        robot = new Robot(hardwareMap);
        robot.configureMotorsZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        gamepad1.runLedEffect(GP.RedLights);
        robot.aiming = true;
        while (opModeIsActive()) {
            double x = gamepad1.left_stick_x;
            double y = - gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;
            double heading = robot.odo.getHeading(AngleUnit.DEGREES);

            if (gamepad1.squareWasPressed()) {
                intakeSpeed = (intakeSpeed == 1 ? 0 : 1);
                robot.intake.setPower(intakeSpeed);
            }

            if (gamepad2.rightBumperWasPressed()) {
                robot.shoot();
            }

            if (gamepad2.leftBumperWasPressed()) {
                robot.turnToAngle(heading);
                robot.toggleAim();
            }

            if (gamepad2.dpadDownWasPressed()) {
                robot.odo.setPosition(teamColor == TEAMCOLOR.RED ? Robot.RedCorner : Robot.BlueCorner);
            }

            if (gamepad1.psWasPressed() || gamepad2.psWasPressed()) {
                teamColor = teamColor == TEAMCOLOR.RED ? TEAMCOLOR.BLUE : TEAMCOLOR.RED;
                gamepad1.runLedEffect(teamColor == TEAMCOLOR.RED ? GP.RedLights : GP.BlueLights);
                gamepad2.runLedEffect(teamColor == TEAMCOLOR.RED ? GP.RedLights : GP.BlueLights);
            }

            robot.mecanumDrive(x, y, rx);
            robot.update(teamColor);
            telemetryM.addData("RPM", robot.RPM);
            telemetryM.addData("Team Color", (teamColor == TEAMCOLOR.RED) ? ("Red") : ("Blue"));
            telemetryM.addData("heading", robot.odo.getHeading(AngleUnit.DEGREES));
            telemetryM.addData("turret rotation", robot.getTurretRotation());
            telemetryM.addData("angle",robot.angle);
            telemetryM.update(telemetry);
        }
    }
}
