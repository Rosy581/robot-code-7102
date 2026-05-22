package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Config;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.Robot.*;


@TeleOp(name = "Main TeleOP", group = "Tele")

public class MainTele extends LinearOpMode {
    Robot robot;
    private TEAMCOLOR teamColor = TEAMCOLOR.RED;
    public DcMotor intake;
    @Override
    public void runOpMode() {
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        robot = new Robot(hardwareMap, OPMODETYPE.TELE);
        robot.configOdo();
        robot.configureDriveTrainZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
        intake = hardwareMap.dcMotor.get(Config.partNames.Intake);
        waitForStart();
        double intakeSpeed = 0;
        while (opModeIsActive()) {
            if(gamepad1.squareWasPressed()){
                intakeSpeed = (intakeSpeed == 1) ? 0 : 1;
                intake.setPower(intakeSpeed);
            }

            if(gamepad1.rightBumperWasPressed()){
                robot.shooting = !robot.shooting;
            }

            if(!gamepad1.atRest()) {
                double x = gamepad1.left_stick_x;
                double y = -gamepad1.left_stick_y;
                double rx = gamepad1.right_stick_x;
                robot.mecanumDrive(x, y, rx);
            }
            // Robot input/actions
            robot.update(teamColor);
            telemetryM.update(telemetry);
        }
    }
}
