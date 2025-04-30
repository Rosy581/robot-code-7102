package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.CRServo;
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
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.hardware.Slide;

//THis is a sample using ideas from https://github.com/gearsincorg/SimplifiedOdometry

@Autonomous
public class DoubleThangInTheBag extends LinearOpMode {
    double startingX = 33;
    double startingY = 7.5;
    double startingHeading = 0;
    
    int floor = 2000;
    int basket;
    
    private static final double DRIVE_GAIN          = 0.05;     // Strength of axial position control
    private static final double DRIVE_ACCEL         = 5;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    private static final double DRIVE_TOLERANCE     = 1;     // Controller is is "inPosition" if position error is < +/- this amount
    private static final double DRIVE_DEADBAND      = 0.2;     // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
    private static final double DRIVE_MINVELOCITY      = 10;     
    private static final double DRIVE_MAXVELOCITY      = 60;     
    private static final double DRIVE_DISTANCENEEDEDTOSTOP      = 30;     

    public ProportionalControl driveController     = new ProportionalControl(DRIVE_GAIN, DRIVE_ACCEL, DRIVE_TOLERANCE, DRIVE_DEADBAND, false);
    public VelocityControl velController     = new VelocityControl(DRIVE_ACCEL, DRIVE_TOLERANCE, DRIVE_DEADBAND, DRIVE_MINVELOCITY, DRIVE_MAXVELOCITY, DRIVE_DISTANCENEEDEDTOSTOP);
    private ElapsedTime holdTimer = new ElapsedTime();  // User for any motion requiring a hold time or timeout.
    private ElapsedTime runTime   = new ElapsedTime();

    SparkFunOTOS odometry;
    private IMU imu;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private Slide slide;
    
    private DcMotor slideTuah;
    private CRServo clawServo;
    private DcMotor backArm1;
    private DcMotor backArm2;  
    private Servo assServo;
    private Servo clawTuah;
    private Servo freakWrist;
    private Datalog datalog;
    private VoltageSensor battery;
    private double desiredHeadingDegrees = 0;
    double autonomousMaxSpeed = 1.0;
    double autonomousMinSpeed = 0.25;
    private double currentrx = 0;
    private double currentRotation = 0;


    @Override
    public void runOpMode() throws InterruptedException{
        odometry        = hardwareMap.get(SparkFunOTOS.class, "otos");
        backLeftMotor   = hardwareMap.get(DcMotor.class, "leftBack");
        backRightMotor  = hardwareMap.get(DcMotor.class, "rightBack");
        frontLeftMotor  = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
        backArm1        = hardwareMap.get(DcMotor.class, "backArm1");
        backArm2        = hardwareMap.get(DcMotor.class, "backArm2");
        battery         = hardwareMap.voltageSensor.get("Control Hub");
        slide           = new Slide(hardwareMap, this, 50);
        datalog         = new Datalog("datalog_03");
        assServo        = hardwareMap.servo.get("assClaw");
        clawTuah        = hardwareMap.servo.get("clawTuah");
        freakWrist      = hardwareMap.servo.get("freakWrist");
        
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backArm1.setDirection(DcMotorSimple.Direction.REVERSE);
        
        backArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        backArm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
        imu.resetYaw();
    
        configureOtos(startingX, startingY, startingHeading);
        
        waitForStart();
        if (opModeIsActive()) {
            SparkFunOTOS.Pose2D pos = odometry.getPosition();
            assServo.setPosition(0.3);
            DriveToPose2D( 15, 15, -45, autonomousMaxSpeed, autonomousMinSpeed, 0.1);
            slide.moveTo(3500);
            while(!slide.atTarget()){}
            assServo.setPosition(0);
            DriveToPose2D( 24.5, 19, -45, autonomousMaxSpeed, autonomousMinSpeed, 0.1);
            moveArm(floor);
            
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

    private void VelDriveToPose2D(double x, double y, double Heading, double MaxPower, double minPower, double holdTime) {
        SparkFunOTOS.Pose2D pos = odometry.getPosition();
        SparkFunOTOS.Pose2D vel = odometry.getVelocity();
        double offsetX = x - pos.x;
        double offsetY = y - pos.y;
        double distanceFromTarget = Math.sqrt(offsetX * offsetX + offsetY * offsetY);
        double desiredVelocity = 0;
        double currentVelocity = 0;


        boolean inPosition = false; 
        SetHeading(Heading);
        velController.reset(MaxPower);  // achieve desired drive distance
        while (inPosition == false && opModeIsActive()) {
            pos = odometry.getPosition();
            vel = odometry.getVelocity();
            offsetX = x - pos.x;
            offsetY = y - pos.y;
            distanceFromTarget = Math.sqrt(offsetX * offsetX + offsetY * offsetY);
            currentVelocity = Math.sqrt(vel.x * vel.x + vel.y * vel.y);
            double desiredPower = velController.getOutput(distanceFromTarget, currentVelocity);
            desiredPower = Math.max(desiredPower, minPower);
            double denominator = Math.max(Math.abs(offsetX) + Math.abs(offsetY), 1);
            denominator = Math.max(Math.abs(offsetX), Math.abs(offsetY));
            denominator = Math.max(denominator, 1);
            double forwardPower = desiredPower * offsetY / denominator;
            double sidewaysPower = desiredPower * offsetX / denominator;
            Drive(sidewaysPower, forwardPower, 0, true);
            telemetry.addData("X velocity", vel.x);
            telemetry.addData("Y velocity", vel.y);
            telemetry.addData("currentVelocity", currentVelocity);

            telemetry.addData("X coordinate", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("Heading angle", pos.h);
            telemetry.addData("desiredPower", desiredPower);
            telemetry.addData("distanceFromTarget", distanceFromTarget);
            telemetry.addData("RotationNeededDegrees", RotationNeededDegrees());
            telemetry.addData("DesiredHeading", DesiredHeading());
            telemetry.addData("Current Heading", CurrentHeading());
            telemetry.update();
            if (velController.inPosition() && Math.abs(RotationNeededDegrees()) < 2) {
                if (holdTimer.time() > holdTime) {
                    inPosition = true;
                }
            } else {
                holdTimer.reset();
            }
            sleep(5);
        } 
        StopBot();
    }
    private void DriveToPose2D(double x, double y, double Heading, double MaxPower, double minPower, double holdTime) {
        SparkFunOTOS.Pose2D pos = odometry.getPosition();
        double offsetX = x - pos.x;
        double offsetY = y - pos.y;
        double distanceFromTarget = Math.sqrt(offsetX * offsetX + offsetY * offsetY);
        boolean inPosition = false;
        SetHeading(Heading);
        driveController.reset(MaxPower);  // achieve desired drive distance
        while (inPosition == false && opModeIsActive()) {
            pos = odometry.getPosition();
            offsetX = x - pos.x;
            offsetY = y - pos.y;
            distanceFromTarget = Math.sqrt(offsetX * offsetX + offsetY * offsetY);
            double desiredPower = driveController.getOutput(distanceFromTarget);
            desiredPower = Math.max(desiredPower, minPower);
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
            
            datalog.elapsedTime.set(runTime.time());
            datalog.targetX.set(x);
            datalog.targetY.set(y);
            datalog.posX.set(pos.x);
            datalog.posY.set(pos.y);
            datalog.yaw.set(pos.h);
            datalog.battery.set(battery.getVoltage());
            
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

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, -5.625, 180);
        odometry.setOffset(offset);
        odometry.setLinearScalar(1.0);
        odometry.setAngularScalar(1.0);
        odometry.calibrateImu();
        odometry.resetTracking();
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(startingX, startingY, startingHeading);
        odometry.setPosition(currentPosition);

    }
    
    public static class Datalog {
        // The underlying datalogger object - it cares only about an array of loggable fields
        private final Datalogger datalogger;

        // These are all of the fields that we want in the datalog.
        // Note that order here is NOT important. The order is important in the setFields() call below
        public Datalogger.GenericField elapsedTime  = new Datalogger.GenericField("elapsedTime");
        public Datalogger.GenericField targetY      = new Datalogger.GenericField("targetX");
        public Datalogger.GenericField targetX      = new Datalogger.GenericField("targetY");
        public Datalogger.GenericField posX         = new Datalogger.GenericField("posX");
        public Datalogger.GenericField posY         = new Datalogger.GenericField("posY");
        public Datalogger.GenericField powerX       = new Datalogger.GenericField("powerX");
        public Datalogger.GenericField powerY       = new Datalogger.GenericField("powerY");
        public Datalogger.GenericField yaw          = new Datalogger.GenericField("Heading");
        public Datalogger.GenericField battery      = new Datalogger.GenericField("Battery");

        public Datalog(String name) {
            // Build the underlying datalog object
            datalogger = new Datalogger.Builder()

                    // Pass through the filename
                    .setFilename(name)

                    // Request an automatic timestamp field
                    .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)

                    // Tell it about the fields we care to log.
                    // Note that order *IS* important here! The order in which we list
                    // the fields is the order in which they will appear in the log.
                    .setFields(
                            elapsedTime,
                            targetX,
                            targetY,
                            posX,
                            posY,
                            powerX,
                            powerY,
                            yaw,
                            battery
                    )
                    .build();
        }

        // Tell the datalogger to gather the values of the fields
        // and write a new line in the log.
        public void writeLine() {
            datalogger.writeLine();
        }
    }

}
