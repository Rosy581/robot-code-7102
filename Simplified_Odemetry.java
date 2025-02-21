package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.util.Range;

//THis is a sample using ideas from https://github.com/gearsincorg/SimplifiedOdometry

@Autonomous
public class Simplified_Odemetry extends LinearOpMode {
    double startingX = 0;
    double startingY = 0;
    double startingHeading = 0;

    private static final double DRIVE_GAIN          = 0.05;    // Strength of axial position control
    private static final double DRIVE_ACCEL         = .7;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
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
    double autonomousMaxSpeed = 1;
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
            DriveToPose2D(0, 31, 0, autonomousMaxSpeed, 0.1);
            DriveToPose2D(30, 31, 0, autonomousMaxSpeed, 0.1);
            DriveToPose2D(30, 51, 180, autonomousMaxSpeed, 0.1);
            DriveToPose2D(40, 51, 180, autonomousMaxSpeed, 0.1);
            DriveToPose2D(40, 5, 0, autonomousMaxSpeed, 0.1);
            pos = odometry.getPosition();
            while (opModeIsActive()) {
                telemetry.addData("Status", "Done");
                telemetry.update();
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
            desiredPower = Math.max(desiredPower, 0.15);
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

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        //odometry.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        //SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(startingX, startingY, startingHeading);
        //odometry.setPosition(currentPosition);
    }
    
    private void configureOtos(double startingX, double startingY, double startingHeading) {
        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        odometry.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        odometry.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        //SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-4.3125, -1.9375, 90);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, -8.5, 0);
        odometry.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        odometry.setLinearScalar(1.0);
        odometry.setAngularScalar(1.0);
 
        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        odometry.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        odometry.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
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
            if (Math.abs(lastError - error) <= 0.1 && inPosition == false) {
                output = Math.max(output, 0.2);                
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

