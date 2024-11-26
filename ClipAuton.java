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
import org.firstinspires.ftc.teamcode.hardware.Slide;
import org.firstinspires.ftc.teamcode.hardware.Drive;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Clip-Tuah (1)", group = "Robot")

public class ClipAuton extends LinearOpMode {
    private IMU imu;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private CRServo claw;
    private DcMotor slide;
    private Slide slIde;
    private Drive robot;
    Deadline rateLimit = new Deadline(250, TimeUnit.MILLISECONDS);
    
    static final double DRIVE_SPEED = 0.5;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backRightMotor = hardwareMap.dcMotor.get("rightBack");
        claw = hardwareMap.crservo.get("claw");
        slide = hardwareMap.dcMotor.get("slide");
        imu = hardwareMap.get(IMU.class, "imu");
        slIde = new Slide(slide, this);
        robot = new Drive(backLeftMotor, frontLeftMotor, backRightMotor, frontRightMotor, imu, this);

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        telemetry.addData("Yaw (Z)", "%.1f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Starting at", "%7d :%7d :%7d :%7d",
                frontLeftMotor.getCurrentPosition(),
                backLeftMotor.getCurrentPosition(),
                backRightMotor.getCurrentPosition(),
                frontRightMotor.getCurrentPosition());
        telemetry.update();
        String state = "Begin";
        waitForStart();
        while (opModeIsActive() && state != "finished") {
            switch(state){
                case "Begin":
                    claw.setPower(-0.25);
                    slide.setTargetPosition(2750);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(1); 
                    state = "step I";
                    
                    break;
                case "step I":
                    robot.encoderDrive(0.35, 12.0, 12.0);
                    
                    state = "step II";
                    telemetry.addData("HERE","I AM");
                    break;
                case "step II":
                    if(!slide.isBusy()){
                        state = "step III";
                        robot.encoderDrive(0.5, -0.75,-0.75);
                    }
                    break;
                case "step III":
                    slide.setTargetPosition(2250);
                    state = "step IV";
                    rateLimit.reset();
                    break;
                case "step IV":
                    if(!slide.isBusy() && rateLimit.hasExpired()){
                        state = "step V";
                    }
                    break;
                case "step V":
                    claw.setPower(1);
                    robot.encoderDrive(1,-2.0,-2.0);
                    state = "step VI";
                    break;
                case "step 6":
                    claw.setPower(1);
                    robot.encoderDrive(DRIVE_SPEED, -2.3, -2.3);
                    state = "step 7";
                    break;
                case "step 7":
                    slide.setTargetPosition(0);
                    robot.encoderDrive(DRIVE_SPEED/2, -12, -12);
                    state = "step 8";
                    break;
                case "step 8":
                    robot.encoderDrive(DRIVE_SPEED, -5.0, 5.0);
                    state = "step 9";
                    break;
                case "step 9":
                    robot.moveRight(1, 24);
                    break;
                }   
                telemetry.addData("Step", state);
            
            telemetry.update();
        }
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1250);
    }   
}

