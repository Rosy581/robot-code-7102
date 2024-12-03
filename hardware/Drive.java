package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Drive {
    public static final double COUNTS_PER_MOTOR_REV = 1440; // eg: TETRIX Motor Encoder
    public static final double DRIVE_GEAR_REDUCTION = 1.0; // No External Gearing.
    public static final double WHEEL_DIAMETER_INCHES = 3.77953; // For figuring circumference
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double DRIVE_SPEED = 0.5;
    public static final double TURN_SPEED = 0.5;
    
    private DcMotor frontRightMotor; 
    private DcMotor backRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private LinearOpMode opMode;
    private IMU imu;
    
    public Drive(DcMotor backLeftMotor, DcMotor frontLeftMotor, DcMotor backRightMotor, DcMotor frontRightMotor, IMU imu, LinearOpMode opMode) {
        this.backLeftMotor = backLeftMotor;
        this.frontLeftMotor = frontLeftMotor;
        this.backRightMotor = backRightMotor;
        this.frontRightMotor = frontRightMotor;
        this.imu = imu;
        this.opMode = opMode;
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
    }

    public void rotate(double angle, double speed) {
        double init = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double dist = 0;
        
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(-speed);
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(-speed);
        
        while(opMode.opModeIsActive() && dist >= angle) {
            dist = Math.abs(init - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            opMode.telemetry.addData("dist", dist);
            opMode.telemetry.update();
        }
        
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
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

        frontLeftMotor.setPower(Math.abs(speed));
        frontRightMotor.setPower(Math.abs(speed));
        backLeftMotor.setPower(Math.abs(speed));
        backRightMotor.setPower(Math.abs(speed));

        while (opMode.opModeIsActive() &&
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

        frontLeftMotor.setPower(Math.abs(speed));
        frontRightMotor.setPower(Math.abs(speed));
        backLeftMotor.setPower(Math.abs(speed));
        backRightMotor.setPower(Math.abs(speed));

        while (opMode.opModeIsActive() &&
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

    public void encoderDrive(double speed,
            double leftInches, double rightInches) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        if (opMode.opModeIsActive()) {

            newFrontLeftTarget  = frontLeftMotor.getCurrentPosition()  - (int) (leftInches  * COUNTS_PER_INCH);
            newFrontRightTarget = frontRightMotor.getCurrentPosition() - (int) (rightInches * COUNTS_PER_INCH);
            newBackLeftTarget   = backLeftMotor.getCurrentPosition()   - (int) (leftInches  * COUNTS_PER_INCH);
            newBackRightTarget  = backRightMotor.getCurrentPosition()  - (int) (rightInches * COUNTS_PER_INCH);

            frontLeftMotor.setTargetPosition(newFrontLeftTarget);
            frontRightMotor.setTargetPosition(newFrontRightTarget);
            backLeftMotor.setTargetPosition(newBackLeftTarget);
            backRightMotor.setTargetPosition(newBackRightTarget);

            frontLeftMotor.setMode( DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftMotor.setMode(  DcMotor.RunMode.RUN_TO_POSITION);
            backRightMotor.setMode( DcMotor.RunMode.RUN_TO_POSITION);

            frontLeftMotor.setPower(Math.abs(speed));
            frontRightMotor.setPower(Math.abs(speed));
            backLeftMotor.setPower(Math.abs(speed));
            backRightMotor.setPower(Math.abs(speed));

            while (opMode.opModeIsActive() & (frontLeftMotor.isBusy() &&
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
    }
} 