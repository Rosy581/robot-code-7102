package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Robot;


@TeleOp(name = "OUTREACH USE THIS", group = "Tele")

public class OUTREACH extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);
        robot.configureMotorsZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.aiming = false;
        waitForStart();
        while (opModeIsActive()) {
            double x = gamepad1.left_stick_x;
            double y = - gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;
            robot.mecanumDrive(x, y, rx);
        }
    }
}
