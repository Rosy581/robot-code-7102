package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.GP;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.Robot.TEAMCOLOR;


@TeleOp(name = "Main TeleOP", group = "Tele")

public class TeleDrive extends LinearOpMode {
    Robot robot;
    double angle = 0;
    private TEAMCOLOR teamColor = TEAMCOLOR.RED;

    @Override
    public void runOpMode() {
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        robot = new Robot(hardwareMap);
        robot.configOdo();
        robot.configureMotorsZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        gamepad1.runLedEffect(GP.RedLights);
        robot.aiming = false;
        robot.setPosition(new Pose2D(DistanceUnit.INCH,89,25,AngleUnit.DEGREES,0));
        while (opModeIsActive()) {
            double x = gamepad1.left_stick_x;
            double y = - gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;
            double heading = robot.getHeading();

            if (gamepad1.squareWasPressed() || gamepad2.squareWasPressed()) {
                robot.toggleIntake();
            }

            if (gamepad1.psWasPressed() || gamepad2.psWasPressed()) {
                teamColor = teamColor == TEAMCOLOR.RED ? TEAMCOLOR.BLUE : TEAMCOLOR.RED;
                gamepad1.runLedEffect(teamColor == TEAMCOLOR.RED ? GP.RedLights : GP.BlueLights);
                gamepad2.runLedEffect(teamColor == TEAMCOLOR.RED ? GP.RedLights : GP.BlueLights);
            }

            if (gamepad1.dpadRightWasPressed()){
                robot.toggleShootingDistance();
            }

            if(gamepad2.circleWasPressed()){
                robot.toggleShootingDistance();
            }

            if (gamepad2.rightBumperWasPressed()|| gamepad1.rightBumperWasPressed()) {
                robot.shoot(); 
            }

            if (gamepad2.leftBumperWasPressed()) {
                robot.turnToAngle(heading);
                robot.toggleAim();
            }

            if (gamepad2.dpadDownWasPressed()) {
                robot.setPosition(teamColor == TEAMCOLOR.RED ? Robot.RedCorner : Robot.BlueCorner);
            }

            if(gamepad2.dpadLeftWasPressed()){
                robot.aiming = false;
                robot.turnToAngle(heading+angle);
                angle += 2;
            }

            if(gamepad2.dpadRightWasPressed()){
                robot.aiming = false;
                robot.turnToAngle(heading+angle);
                angle -= 2;

            }

            if(gamepad2.dpadUpWasPressed()){
                robot.aiming = false;
                robot.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.turretMotor.setTargetPosition(0);
                robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            robot.mecanumDrive(x, y, rx);
            robot.update(teamColor);
            telemetryM.addData("RPM", robot.RPM);
            telemetryM.addData("Team Color", (teamColor == TEAMCOLOR.RED) ? ("Red") : ("Blue"));
            telemetryM.addData("heading", robot.getHeading());
            telemetryM.addData("turret rotation", robot.getTurretRotation());
            telemetryM.addData("angle",robot.angle);
            telemetryM.addData("Shooing",robot.shooting);
            telemetryM.addData("targetRpm",robot.targetRPM);
            telemetryM.addData("x",robot.getPos());
            telemetryM.update(telemetry);
        }
    }
}
