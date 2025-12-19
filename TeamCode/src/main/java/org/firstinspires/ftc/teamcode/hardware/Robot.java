package org.firstinspires.ftc.teamcode.hardware;

import android.util.Size;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.vision.VisionPortal;

import org.firstinspires.ftc.teamcode.configurables.Config.shooterConstants;
import org.firstinspires.ftc.teamcode.configurables.Config.motorNames;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class Robot {
    public static int target = 0;
    public static double lastSeen = 0;
    private final LinearOpMode opMode;
    public Point point = new Point(0, 0);
    public boolean shooting = false;
    public boolean aiming = false;
    public boolean revved = false;
    public boolean onTarget;
    public VisionPortal camView;
    public final static double turretEncoderResolution = 537.7;
    public double RPM = 0;
    public double shooterPower = 0;
    public DcMotorEx turretMotor;
    public Servo aimServo1;
    public Servo aimServo2;
    public Servo kicker;
    private final static float decimation = 2;
    private AprilTagProcessor aprilTag;
    private DcMotorEx shooter;
    public GoBildaPinpointDriver odo;
    private final static double gearRatio = 198 / 48.0;
    private final static int minPos = 1060;
    private final static int maxPos = - minPos;
    private final static Pose2D StartingPos = new Pose2D(DistanceUnit.INCH, 72, 8.5, AngleUnit.DEGREES, 0);
    private Deadline rateLimit = new Deadline(500, TimeUnit.MICROSECONDS);

    public Robot(LinearOpMode _opMode, @NonNull HardwareMap _hardwareMap) {
        turretMotor = _hardwareMap.get(DcMotorEx.class, motorNames.Turret);
        aimServo1 = _hardwareMap.servo.get(motorNames.AimServo1);
        aimServo2 = _hardwareMap.servo.get(motorNames.AimServo2);
        kicker = _hardwareMap.servo.get(motorNames.kickerServo);
        shooter = _hardwareMap.get(DcMotorEx.class, motorNames.Shooter);
        opMode = _opMode;
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setTargetPosition(0);
        turretMotor.setTargetPositionTolerance(15);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        odo = _hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        Size resolution = new Size(1280, 720);

        aprilTag = new AprilTagProcessor.Builder().build();
        camView = new VisionPortal.Builder()
                .setCameraResolution(resolution)
                .setCamera(_hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        aprilTag.setDecimation(decimation);

        odo.setOffsets(0, 6.5, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        odo.setPosition(StartingPos);
    }

    public enum TEAMCOLOR {
        RED,
        BLUE
    }


    public void push() {
        kicker.setPosition(1.0);
        rateLimit.reset();
    }

    public void shoot() {
        shooting = ! shooting;
    }

    public void update(TEAMCOLOR teamcolor) {
        RPM = (shooter.getVelocity() / 28) * 60;
        revved = Math.abs(shooterConstants.targetRPM - RPM) < shooterConstants.tolerance;
        if (rateLimit.hasExpired()) {
            kicker.setPosition(0.0);
        }

        if (shooting) {
            shooter.setVelocity((shooterConstants.targetRPM / 60) * 28);
        } else if (!shooting) {
            shooter.setVelocity(0.0);
        }

        //angle of wall is 125 degrees
        //blue team april tag ID = 20
        //red team april tag ID =24
        // moving the turret counter clockwise is +
        // moving the turret clock wise is -

        if (Math.abs(point.x - 640) > 100) {
            odo.update();
            turretMotor.setPower(1.0);
            onTarget = false;
            int targetedId = teamcolor == TEAMCOLOR.RED ? 24 : 20;
            aiming = true;
            ArrayList<AprilTagDetection> currentDetections = aprilTag.getDetections();
            if (currentDetections.isEmpty()) {
                turnToAngle(lastSeen);
            } else {
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.metadata != null) {
                        if (detection.id == targetedId) {
                            point = detection.center;
                        }
                    }
                }
            }
        } else {
            onTarget = true;
            lastSeen = getTurretRotation();
        }
    }

    public double encoderToRotation(int encoderPos) {
        return ((encoderPos / turretEncoderResolution) * gearRatio) * 360 + odo.getHeading(AngleUnit.DEGREES);
    }

    public double getTurretRotation() {
        return encoderToRotation(turretMotor.getCurrentPosition());
    }

    public int rotationToEncoder(double angle) {
        return (int) ((angle - odo.getHeading(AngleUnit.DEGREES) / 360) / gearRatio * turretEncoderResolution);
    }

    public void turnToAngle(double angle) {
        turretMotor.setTargetPosition(rotationToEncoder(angle));
        turretMotor.setPower(1.0);
    }
}