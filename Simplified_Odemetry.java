package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.util.Range;

//THis is a sample using ideas from https://github.com/gearsincorg/SimplifiedOdometry

@Autonomous
public class Simplified_Odemetry extends LinearOpMode {
    double startingX = 83;
    double startingY = 8.5;
    double startingHeading = 0;

    private static final double DRIVE_GAIN          = 0.025;    // Strength of axial position control
    private static final double DRIVE_ACCEL         = 4;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    private static final double DRIVE_TOLERANCE     = 1;     // Controller is is "inPosition" if position error is < +/- this amount
    private static final double DRIVE_DEADBAND      = 0.2;     // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE

    public ProportionalControl driveController     = new ProportionalControl(DRIVE_GAIN, DRIVE_ACCEL, DRIVE_TOLERANCE, DRIVE_DEADBAND, false);
    private ElapsedTime holdTimer = new ElapsedTime();  // User for any motion requiring a hold time or timeout.

    SparkFunOTOS odometry;
    private IMU imu;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private double desiredHeadingDegrees = 0;
    double autonomousMaxSpeed = 0.8;
    private double currentrx = 0;
    private double currentRotation = 0;


    @Override
    public void runOpMode() {
        odometry = hardwareMap.get(SparkFunOTOS.class, "otos");
        backLeftMotor = hardwareMap.get(DcMotor.class, "leftBack");
        backRightMotor = hardwareMap.get(DcMotor.class, "rightBack");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
    
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        configureOtos(startingX, startingY, startingHeading);
        
        waitForStart();
        if (opModeIsActive()) {
            SparkFunOTOS.Pose2D pos = odometry.getPosition();
            DriveToPose2D(startingX, 38.5, 0, autonomousMaxSpeed, 0.1);
            DriveToPose2D(108, 36, 0, autonomousMaxSpeed, 0.1);
            DriveToPose2D(108, 60, 0, autonomousMaxSpeed, 0.1);
            DriveToPose2D(120, 60, 0, autonomousMaxSpeed, 0.1);
            DriveToPose2D(120, 18, 0, 0.5, 0.1);
            DriveToPose2D(120, 60, 0, autonomousMaxSpeed, 0.1);
            DriveToPose2D(130, 60, 0, autonomousMaxSpeed, 0.1);
            DriveToPose2D(130, 18, 0, 0.5, 0.1);
            pos = odometry.getPosition();
            while (opModeIsActive()) {
                
            }
        }
    }

    public void StopBot() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
    
    private void DriveToPose2D(double x, double y, double Heading, double MaxPower, double holdTime) {
        SparkFunOTOS.Pose2D pos = odometry.getPosition();
        double offsetX = x - pos.x;
        double offsetY = y - pos.y;
        double distanceFromTarget = Math.sqrt(offsetX * offsetX + offsetY * offsetY);
        boolean inPosition = false;
        SetHeading(Heading);
        driveController.reset(MaxPower);   // achieve desired drive distance
        while (inPosition == false && opModeIsActive()) {
            pos = odometry.getPosition();
            offsetX = x - pos.x;
            offsetY = y - pos.y;
            distanceFromTarget = Math.sqrt(offsetX * offsetX + offsetY * offsetY);
            double desiredPower = driveController.getOutput(distanceFromTarget);
            desiredPower = Math.max(desiredPower, 0.25);
            double denominator = Math.max(Math.abs(offsetX) + Math.abs(offsetY), 1);
            double forwardPower = desiredPower * offsetY / denominator;
            double sidewaysPower = desiredPower * offsetX / denominator;
            Drive(sidewaysPower, forwardPower, 0, true);
            telemetry.addData("X coordinate", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("Heading angle", pos.h);
            telemetry.addData("desiredPower", desiredPower);
            telemetry.addData("distanceFromTarget", distanceFromTarget);
            telemetry.addData("RotationNeededDegrees", RotationNeededDegrees());
            telemetry.addData("DesiredHeading", DesiredHeading());
            telemetry.addData("Current Heading", CurrentHeading());
            telemetry.update();
            if (driveController.inPosition() && Math.abs(RotationNeededDegrees()) < 2) {
                if (holdTimer.time() > holdTime) {
                    inPosition = true;
                }
            } else {
                holdTimer.reset();
            }
            sleep(10);
        } 
        StopBot();
    }

    public void SetHeading(double desiredHeadingDegrees){
        this.desiredHeadingDegrees = desiredHeadingDegrees;
    }

    public double DesiredHeading() {
        return desiredHeadingDegrees;
    }


    public double CurrentHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public double RotationNeededDegrees(){
        double botHeadingDegrees = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double botRotationNeeded = desiredHeadingDegrees - botHeadingDegrees;
        if (botRotationNeeded > 180) {botRotationNeeded = botRotationNeeded - 360;}
        if (botRotationNeeded < -180) {botRotationNeeded = botRotationNeeded + 360;}
        return botRotationNeeded;
    }

    
    public void Drive(double sideways, double forward, double rotation, boolean fieldCentric) {
        double rx = rotation;
        if (rotation != (double)0) {
            currentRotation = Math.abs(rotation);
        } else {
            double botRotationNeeded = RotationNeededDegrees();
            if (botRotationNeeded != (double)0) {
                double botPitch = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
                if (botPitch <= 5) {
                    rx = Math.min(1, (double)(Math.abs(botRotationNeeded)) / 90);
                    rx = Math.min(rx, Math.abs(currentrx) + 0.1);
                    if ((double)(Math.abs(botRotationNeeded)) >= 1) {rx = Math.max(rx, 0.15);}
                    rx = Math.signum(botRotationNeeded) * -rx;
                }
            }
        }
        currentrx = rx;
        double frontLeftPower = 0;
        double backLeftPower = 0;
        double frontRightPower = 0;
        double backRightPower = 0;
        if (fieldCentric == true) {
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            // Rotate the movement direction counter to the bot's rotation
            double rotX = sideways * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
            double rotY = sideways * Math.sin(-botHeading) + forward * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            frontLeftPower = (rotY + rotX + rx) / denominator;
            backLeftPower = (rotY - rotX + rx) / denominator;
            frontRightPower = (rotY - rotX - rx) / denominator;
            backRightPower = (rotY + rotX - rx) / denominator;
        } else {
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(forward) + Math.abs(sideways) + Math.abs(rx), 1);
            frontLeftPower = (forward + sideways + rx) / denominator;
            backLeftPower = (forward - sideways + rx) / denominator;
            frontRightPower = (forward - sideways - rx) / denominator;
            backRightPower = (forward + sideways - rx) / denominator;
        }
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
        backLeftMotor.setPower(backLeftPower);
        if (currentRotation > 0) {
            currentRotation -= 0.1;
            SetHeading(CurrentHeading());
        }
    }


    
    private void resetOtos(double startingX, double startingY, double startingHeading) {
        odometry.calibrateImu();
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(startingX, startingY, startingHeading);
        odometry.setPosition(currentPosition);
    }
    
    private void configureOtos(double startingX, double startingY, double startingHeading) {
        odometry.setLinearUnit(DistanceUnit.INCH);
        odometry.setAngularUnit(AngleUnit.DEGREES);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, -8.5, 0);
        odometry.setOffset(offset);
        odometry.setLinearScalar(1.0);
        odometry.setAngularScalar(1.0);
        odometry.calibrateImu();
        odometry.resetTracking();
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(startingX, startingY, startingHeading);
        odometry.setPosition(currentPosition);

    }
    
}

class ProportionalControl {
    double  lastOutput;
    double  lastError;
    double  gain;
    double  accelLimit;
    double  liveOutputLimit;
    double  tolerance;
    double deadband;
    boolean circular;
    boolean inPosition;
    ElapsedTime cycleTime = new ElapsedTime();

    public ProportionalControl(double gain, double accelLimit, double tolerance, double deadband, boolean circular) {
        this.gain = gain;
        this.accelLimit = accelLimit;
        this.tolerance = tolerance;
        this.deadband = deadband;
        this.circular = circular;
        reset();
    }

    /**
     * Determines power required to obtain the desired setpoint value based on new input value.
     * Uses proportional gain, and limits rate of change of output, as well as max output.
     * @param input  Current live control input value (from sensors)
     * @return desired output power.
     */
    public double getOutput(double error) {
        //double error = setPoint - input;
        double dV = cycleTime.seconds() * accelLimit;
        double output;

        // normalize to +/- 180 if we are controlling heading
        if (circular) {
            while (error > 180)  error -= 360;
            while (error <= -180) error += 360;
        }

        inPosition = (Math.abs(error) < tolerance);

        // Prevent any very slow motor output accumulation
        if (Math.abs(error) <= deadband) {
            output = 0;
        } else {
            // calculate output power using gain and clip it to the limits
            output = (error * gain);
            output = Range.clip(output, -liveOutputLimit, liveOutputLimit);

            // Now limit rate of change of output (acceleration)
            if ((output - lastOutput) > dV) {
                output = lastOutput + dV;
            } else if ((output - lastOutput) < -dV) {
                output = lastOutput - dV;
            }
        }

        lastOutput = output;
        lastError = error;
        cycleTime.reset();
        return output;
    }

    public boolean inPosition(){
        return inPosition;
    }

    public void reset(double powerLimit) {
        liveOutputLimit = Math.abs(powerLimit);
        reset();
    }

    /**
     * Leave everything else the same, Just restart the acceleration timer and set output to 0
     */
    public void reset() {
        cycleTime.reset();
        inPosition = false;
        lastOutput = 0.0;
    }
}

