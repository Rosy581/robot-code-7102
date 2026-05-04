package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.RobotBase.*;


@TeleOp(name = "Main TeleOP", group = "Tele")

public class TeleDrive extends LinearOpMode {
    Robot robot;
    private TEAMCOLOR teamColor = TEAMCOLOR.RED;

    @Override
    public void runOpMode() {
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        robot = new Robot(hardwareMap, OPMODETYPE.TELE);
        robot.configOdo();
        robot.configureDriveTrainZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

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
