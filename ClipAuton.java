package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.hardware.Drive;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Clip that THANG", group = "Robot")

public class ClipAuton extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private CRServo claw;
    private DcMotor slide;
    private Drive robot;
    Deadline rateLimit = new Deadline(250, TimeUnit.MILLISECONDS);

    static final double DRIVE_SPEED = 0.5;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        claw = hardwareMap.crservo.get("claw");
        slide = hardwareMap.dcMotor.get("slide");
        robot = new Drive(hardwareMap, this);

        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();

        telemetry.addData("Yaw (Z)", "%.1f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.update();
        String state = "Begin";
        waitForStart();
        while (opModeIsActive() && state != "finished") {
            switch (state) {
                case "Begin":
                    claw.setPower(-0.25);
                    slide.setTargetPosition(2750);
                    robot.moveArm(450);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(1);
                    state = "step 1";
                    break;
                case "step 1":
                    robot.encoderDrive(0.35, 12.0, 12.0);
                    state = "step 2";
                    break;
                case "step 2":
                    if (!slide.isBusy()) {
                        robot.encoderDrive(0.5, -0.75, -0.75);
                        state = "step 3";
                    }
                    break;
                case "step 3":
                    slide.setTargetPosition(2250);
                    state = "step 4";
                    rateLimit.reset();
                    break;
                case "step 4":
                    if (!slide.isBusy() && rateLimit.hasExpired()) {
                        state = "step 5";
                    }
                    break;
                case "step 5":
                    claw.setPower(1);
                    robot.encoderDrive(1, -2.0, -2.0);
                    state = "step 6";
                    break;
                case "step 6":
                    robot.encoderDrive(0.75, -2.3, -2.3);
                    state = "step 7";
                    break;
                case "step 7":
                    robot.rotateTo(90, -0.25);
                    robot.encoderDrive(1, 15);
                    robot.setPower(.25);
                    state = "step 8";
                    break;
                case "step 8":
                    if (robot.isTouched()) {
                        robot.setPower(0);
                        state = "step 9";
                    }
                    break;
                case "step 9":
                    robot.moveRight(0.5,10); 
                    state = "finished";  
                    break;
            }
        }
    }
}