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

@Autonomous(name="Just put that THANG in the bag")

public class JustPutThatThangInTheBag extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private CRServo claw;
    private DcMotor slide;
    private Drive robot;
    Deadline rateLimit = new Deadline(250, TimeUnit.MILLISECONDS);

    static final double DRIVE_SPEED = 0.75;
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
                    robot.encoderDrive(DRIVE_SPEED,8);
                    robot.rotateTo(130,0.5);
                    slide.setTargetPosition(3500);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(1);
                    state = "step 1";
                    break;
                case "step 1":
                    robot.encoderDrive(DRIVE_SPEED/2, 10);  
                    state = "step 2";                    
                    break;
                case "step 2":
                    if(!slide.isBusy()){
                        claw.setPower(0.25);
                        Thread.sleep(750);
                        state = "step 3";
                    }
                    break;
                case "step 3":
                    robot.encoderDrive(DRIVE_SPEED,-3);
                    robot.rotateTo(0,-0.6);
                    robot.encoderDrive(DRIVE_SPEED,12);
                    state = "step 4";
                    break;
                case "step 4":
                    robot.rotateTo(90,-0.5);
                    robot.encoderDrive(0.25,10);
                    break;
            }
        }    
    }
}