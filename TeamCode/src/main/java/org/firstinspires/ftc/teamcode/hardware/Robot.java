package org.firstinspires.ftc.teamcode.hardware;

import android.util.Size;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.configurables.Config.shooterConstants;
import org.firstinspires.ftc.teamcode.configurables.Config.partNames;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Point;

import java.util.ArrayList;

public class Robot {
    public double lastSeen = 0;
    public double angle = 0;
    public Point point = new Point(0, 0);
    public boolean shooting = false;
    public boolean aiming = false;
    public boolean revved = false;
    public boolean turning = false;
    public boolean onTarget;
    public VisionPortal camView;
    public double RPM = 0;
    public DcMotorEx turretMotor;
    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor backRight;
    public DcMotor backLeft;
    public DcMotor intake;
    public Servo aimServo1;
    public Servo aimServo2;
    public Servo blocker;
    private final static float decimation = 2;
    private AprilTagProcessor aprilTag;
    private DcMotorEx shooter;
    public GoBildaPinpointDriver odo;

    public final static double turretEncoderResolution = 537.7;
    private final static double gearRatio = 35.0 / 110.0;
    private final static Pose2D RedTarget = new Pose2D(DistanceUnit.INCH, 138, 138, AngleUnit.DEGREES, 0);
    private final static Pose2D BlueTarget = new Pose2D(DistanceUnit.INCH, 0, 138, AngleUnit.DEGREES, 0);
    public final static Pose2D RedCorner = new Pose2D(DistanceUnit.INCH, 6.5, 8.5, AngleUnit.DEGREES, 0);
    public final static Pose2D BlueCorner = new Pose2D(DistanceUnit.INCH, 6.5, 8.5, AngleUnit.DEGREES, 0);
    public static final Size resolution = new Size(1280, 720);

    public Robot(@NonNull HardwareMap _hardwareMap) {
        frontRight = _hardwareMap.dcMotor.get(partNames.FrontRight);
        frontLeft = _hardwareMap.dcMotor.get(partNames.FrontLeft);
        backRight = _hardwareMap.dcMotor.get(partNames.BackRight);
        backLeft = _hardwareMap.dcMotor.get(partNames.BackLeft);
        turretMotor = _hardwareMap.get(DcMotorEx.class, partNames.Turret);
        aimServo1 = _hardwareMap.servo.get(partNames.AimServo1);
        aimServo2 = _hardwareMap.servo.get(partNames.AimServo2);
        shooter = _hardwareMap.get(DcMotorEx.class, partNames.Shooter);
        blocker = _hardwareMap.servo.get(partNames.Blocker);
        intake = _hardwareMap.dcMotor.get(partNames.Intake);

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setPower(1);

        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        odo = _hardwareMap.get(GoBildaPinpointDriver.class, partNames.Odometry);


        aprilTag = new AprilTagProcessor.Builder().build();
        camView = new VisionPortal.Builder()
                .setCameraResolution(resolution)
                .setCamera(_hardwareMap.get(WebcamName.class, partNames.Camera))
                .addProcessor(aprilTag)
                .build();

        aprilTag.setDecimation(decimation);

        odo.setOffsets(0, 6.5, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
    }

    public enum TEAMCOLOR {
        RED,
        BLUE
    }

    public void toggleAim(){
        aiming = !aiming;
    }

    public void shoot() {
        shooting = ! shooting;
        if (shooting) {
            blocker.setPosition(0.45);
            intake.setPower(0.0);
        } else {
            blocker.setPosition(0);
        }
    }

    public void update(TEAMCOLOR teamColor) {
        odo.update();

        RPM = (shooter.getVelocity() / 28) * 60;
        revved = Math.abs(shooterConstants.targetRPM - RPM) < shooterConstants.tolerance;

        turning = turretMotor.isBusy();

        if (shooting) {
            shooter.setVelocity((shooterConstants.targetRPM / 60) * 28);
        } else {
            shooter.setVelocity(0.0);
        }
        // moving the turret counter clockwise is +
        // moving the turret clock wise is -
        // cam fov is 78 deg
        if (aiming) {
            int targetedId = teamColor == TEAMCOLOR.RED ? 24 : 20;
            ArrayList <AprilTagDetection> currentDetections = aprilTag.getDetections();
            if (currentDetections.isEmpty()) {
                angle = aimAtPos(teamColor);
            } else {
                for (AprilTagDetection detection : currentDetections) {
                    //noinspection UnnecessaryBoxing
                    if (detection.id == Integer.valueOf(targetedId)) {
                        point = detection.center;
                        onTarget = (Math.abs(point.x - ((double) resolution.getWidth() / 2)) < 50);
                        if (onTarget) {
                            turretMotor.setTargetPosition(turretMotor.getCurrentPosition());
                            lastSeen = getTurretRotation();
                        } else if(!turning){
                            double difference = 38-((point.x / (resolution.getWidth()) * 78.0));
                            turnToAngle(getTurretRotation()-difference);
                        }
                    }
                }
            }
        }
    }

    public double encoderToRotation(int encoderPos) {
        return ((encoderPos / turretEncoderResolution) * gearRatio) * 360 + odo.getHeading(AngleUnit.DEGREES);
    }
    public int rotationToEncoder(double angle) {
        return (int) (((angle - odo.getHeading(AngleUnit.DEGREES)) * turretEncoderResolution) / (360 * gearRatio));
    }

    public double getTurretRotation() {
        return encoderToRotation(turretMotor.getCurrentPosition());
    }
    public double aimAtPos(Pose2D target) {
        Pose2D pos = odo.getPosition();
        double x = Math.abs((target.getX(DistanceUnit.INCH) - pos.getX(DistanceUnit.INCH)));
        double y = Math.abs((target.getY(DistanceUnit.INCH) - pos.getY(DistanceUnit.INCH)));
        double angle = Math.toDegrees(Math.atan(y / x)) * (pos.getX(DistanceUnit.INCH) < x ? -1 : 1) ;
        turnToAngle(angle);
        return angle;
    }

    public double aimAtPos(TEAMCOLOR teamcolor) {
        return aimAtPos(teamcolor == TEAMCOLOR.RED ? RedTarget : BlueTarget);
    }

    public void turnToAngle(double angle) {
        turretMotor.setTargetPosition(rotationToEncoder(angle));
        turretMotor.setPower(1.0);
    }

    public void configureMotorsZeroPower(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        frontLeft.setZeroPowerBehavior(zeroPowerBehavior);
        frontRight.setZeroPowerBehavior(zeroPowerBehavior);
        backLeft.setZeroPowerBehavior(zeroPowerBehavior);
        backRight.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void mecanumDrive(double x, double y, double rx) {
        double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rx), 1);
        double frontRightPower = (y - x - rx) / denominator;
        double frontLeftPower = (y + x + rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;

        frontRight.setPower(frontRightPower);
        frontLeft.setPower(frontLeftPower);
        backRight.setPower(backRightPower);
        backLeft.setPower(backLeftPower);
    }
}