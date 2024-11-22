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

@Autonomous(name = "Clip 2 Of That THANGS", group = "Robot")

public class ClipTwoOfThatTHANGS extends LinearOpMode {
    private IMU imu;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private CRServo claw;
    private DcMotor slide;
    private Slide sLide;

    static final double COUNTS_PER_MOTOR_REV = 1440; // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0; // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 3.77953; // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
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
        //sLide = new Slide(slide);

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

        waitForStart();
        claw.setPower(-0.25);
        slIde.moveTo(7250);
        encoderDrive(0.35,24,24,2.0);
        encoderDrive(DRIVE_SPEED,-0.5,-0.5,1.0);
        slIde.moveTo(6250);
        claw.setPower(1);
        encoderDrive(0.25,-2.3,-2.3,1.0); 
        slIde.moveTo(0);
        //WORK IN PROGRESS STOPS AFTER CLIPPING FIRST ONE
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1250); 
    }

    public void moveRight(double speed, double dist) {

        int newFrontLeftTarget = frontLeftMotor.getCurrentPosition() - (int) (dist * COUNTS_PER_INCH);
        int newFrontRightTarget = frontRightMotor.getCurrentPosition() + (int) (dist * COUNTS_PER_INCH);
        int newBackLeftTarget = backLeftMotor.getCurrentPosition() + (int) (dist * COUNTS_PER_INCH);
        int newBackRightTarget = backRightMotor.getCurrentPosition() - (int) (dist * COUNTS_PER_INCH);

        frontLeftMotor.setTargetPosition(newFrontLeftTarget);
        frontRightMotor.setTargetPosition(newFrontRightTarget);
        backLeftMotor.setTargetPosition(newBackLeftTarget);
        backRightMotor.setTargetPosition(newBackRightTarget);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        frontLeftMotor.setPower(Math.abs(speed));
        frontRightMotor.setPower(Math.abs(speed));
        backLeftMotor.setPower(Math.abs(speed));
        backRightMotor.setPower(Math.abs(speed));

        while (opModeIsActive() &&
                (frontLeftMotor.isBusy() &&
                        frontRightMotor.isBusy() &&
                        backLeftMotor.isBusy() &&
                        backRightMotor.isBusy())) {

        }

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveLeft(double speed, double dist) {

        int newFrontLeftTarget = frontLeftMotor.getCurrentPosition() + (int) (dist * COUNTS_PER_INCH);
        int newFrontRightTarget = frontRightMotor.getCurrentPosition() - (int) (dist * COUNTS_PER_INCH);
        int newBackLeftTarget = backLeftMotor.getCurrentPosition() - (int) (dist * COUNTS_PER_INCH);
        int newBackRightTarget = backRightMotor.getCurrentPosition() + (int) (dist * COUNTS_PER_INCH);

        frontLeftMotor.setTargetPosition(newFrontLeftTarget);
        frontRightMotor.setTargetPosition(newFrontRightTarget);
        backLeftMotor.setTargetPosition(newBackLeftTarget);
        backRightMotor.setTargetPosition(newBackRightTarget);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        frontLeftMotor.setPower(Math.abs(speed));
        frontRightMotor.setPower(Math.abs(speed));
        backLeftMotor.setPower(Math.abs(speed));
        backRightMotor.setPower(Math.abs(speed));

        while (opModeIsActive() &&
                (frontLeftMotor.isBusy() &&
                        frontRightMotor.isBusy() &&
                        backLeftMotor.isBusy() &&
                        backRightMotor.isBusy())) {

        }

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void rotate(double speed,IMU imu, double degrees, boolean right){
        double startAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double orientation;
        double target = startAngle + (right ?degrees : -degrees);
        double power  = (right ? -speed : speed);
        double distMoved = 0;
        double distToMove = Math.abs(startAngle - target);
        if(opModeIsActive()) {
            frontLeftMotor.setPower(-power);
            backLeftMotor.setPower(-power);
            frontRightMotor.setPower(power);
            backRightMotor.setPower(power);
        }
        while(distToMove > distMoved && opModeIsActive()){
            telemetry.addData("Yaw (Z)", "%.1f Deg. (Heading)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("Rotate dist", distMoved);
            distMoved = Math.abs(startAngle - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }
        
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);        
    }   
    
    public void encoderDrive(double speed,
            double leftInches, double rightInches,
            double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        if (opModeIsActive()) {

            newFrontLeftTarget = frontLeftMotor.getCurrentPosition() - (int) (leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = frontRightMotor.getCurrentPosition() - (int) (rightInches * COUNTS_PER_INCH);
            newBackLeftTarget = backLeftMotor.getCurrentPosition() - (int) (leftInches * COUNTS_PER_INCH);
            newBackRightTarget = backRightMotor.getCurrentPosition() - (int) (rightInches * COUNTS_PER_INCH);

            frontLeftMotor.setTargetPosition(newFrontLeftTarget);
            frontRightMotor.setTargetPosition(newFrontRightTarget);
            backLeftMotor.setTargetPosition(newBackLeftTarget);
            backRightMotor.setTargetPosition(newBackRightTarget);
            
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            frontLeftMotor.setPower(Math.abs(speed));
            frontRightMotor.setPower(Math.abs(speed));
            backLeftMotor.setPower(Math.abs(speed));
            backRightMotor.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeftMotor.isBusy() &&
                            frontRightMotor.isBusy() &&
                            backLeftMotor.isBusy() &&
                            backRightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d :%7d :%7d",
                        newFrontLeftTarget,
                        newFrontRightTarget,
                        newBackLeftTarget,
                        newBackRightTarget);
                telemetry.addData("Currently at", " at %7d :%7d :%7d :%7d",
                        frontLeftMotor.getCurrentPosition(),
                        frontRightMotor.getCurrentPosition(),
                        backLeftMotor.getCurrentPosition(),
                        backRightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            // Turn off RUN_TO_POSITION

            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(100); // optional pause after each move.
        }
    }
}
