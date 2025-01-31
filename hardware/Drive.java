package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class Drive {
    public static final double COUNTS_PER_MOTOR_REV = 1440; // eg: TETRIX Motor Encoder
    public static final double DRIVE_GEAR_REDUCTION = 1.0; // No External Gearing.
    public static final double WHEEL_DIAMETER_INCHES = 3.77953; // For figuring circumference
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double DRIVE_SPEED = 0.5;
    public static final double TURN_SPEED = 0.5;
    
    public DcMotor frontRightMotor; 
    public DcMotor backRightMotor;
    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor backArm1;
    public DcMotor backArm2;
    private LinearOpMode opMode;
    public DigitalChannel touchSensor;
    public IMU imu;
    
    public Drive(HardwareMap hardwareMap,LinearOpMode opMode) {
        this.frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        this.backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        this.frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        this.backRightMotor = hardwareMap.dcMotor.get("rightBack");
        this.backArm1 = hardwareMap.dcMotor.get("backArm1");
        this.backArm2 = hardwareMap.dcMotor.get("backArm2");
        this.imu = hardwareMap.get(IMU.class, "imu");
        this.opMode = opMode;
        this.touchSensor = hardwareMap.get(DigitalChannel.class, "touchMe");

        touchSensor.setMode(DigitalChannel.Mode.INPUT);

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backArm1.setDirection(DcMotorSimple.Direction.REVERSE);
        
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backArm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backArm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backArm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }

    public boolean isTouched(){
        return !touchSensor.getState();
    }

    public void rotateTo(double target){
        rotateTo(target,0.5);
    }

    public void rotateTo(double target, double speed) {
        target = Math.abs(target);

        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(-speed);
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(-speed);
        
        while(opMode.opModeIsActive() && (target+1 <= Math.abs(getHeading()) || Math.abs(getHeading()) <= target -1)) {
        }
        
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
    
    public void moveArm (int target){
        backArm1.setTargetPosition(target);
        backArm2.setTargetPosition(target);
        
        backArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backArm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        backArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        backArm1.setPower(1);
        backArm2.setPower(1);
        }

    public void setPower(double n1, double n2){
        setPower(n1,n2,n1,n2);
    }
    
    public void setPower(double n) throws InterruptedException {
        setPower(n,n,n,n);
        if(n == 0){
            Thread.sleep(100);
        }
    }
    
    public void setPower(double n1, double n2, double n3, double n4){
        n1 = -n1;
        n2 = -n2;
        n3 = -n3;
        n4 = -n4;
        frontLeftMotor.setPower(n1);
        frontRightMotor.setPower(n2);
        backLeftMotor.setPower(n3);
        backRightMotor.setPower(n4);
    }
    
    public double getHeading(){
        return getHeading(AngleUnit.DEGREES);
    }

    public double getHeading(AngleUnit unit){
        return imu.getRobotYawPitchRollAngles().getYaw(unit);
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
    
    public void encoderDrive(double speed, double dist) {
        encoderDrive(speed,dist,dist);
    }
    
    public void strafeDrive(double x, double y, double rx){
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            
            double frontLeftPower  = ((y + x + rx) / denominator);
            double backLeftPower   = ((y - x + rx) / denominator);
            double frontRightPower = ((y - x - rx) / denominator);
            double backRightPower  = ((y + x - rx) / denominator);

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
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