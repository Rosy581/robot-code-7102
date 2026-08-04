package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.Robot.*;


@TeleOp(name = "OUTREACH USE THIS", group = "Tele")

public class Outreach extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, OPMODETYPE.TEST);
        robot.configureDriveTrainZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        while (opModeIsActive()) {
            double x = gamepad1.left_stick_x*0.5;
            double y = - gamepad1.left_stick_y*0.5;
            double rx = gamepad1.right_stick_x*0.5;

            if(gamepad1.bWasPressed()){
                robot.toggleIntake();
            }

            robot.mecanumDriveFieldCentric(x, y, rx);

        }
    }
}
