package org.firstinspires.ftc.teamcode.hardware;

import android.util.Size;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.vision.VisionPortal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.configurables.Config.shooterConstants;
import org.firstinspires.ftc.teamcode.configurables.Config.motorNames;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class Robot {
    public static int target = 0;
    public Point point = new Point(0,0);
    public boolean shooting = false;
    public boolean aiming = false;
    public boolean revved = false;
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
    private GoBildaPinpointDriver odo;
    private final static double gearRatio = 198 / 48.0;
    private final static int minPos = 1060;
    private final static int maxPos = - minPos;
    private final static Pose2D StartingPos = new Pose2D(DistanceUnit.INCH, 72, 8.5, AngleUnit.DEGREES, 0);
    private final static Pose2D BluePos = new Pose2D(DistanceUnit.INCH, 9, 135, AngleUnit.DEGREES, 0);
    private final static Pose2D RedPos = new Pose2D(DistanceUnit.INCH, 135, 135, AngleUnit.DEGREES, 0);
    private Deadline rateLimit = new Deadline(1, TimeUnit.SECONDS);

    public Robot(LinearOpMode _opMode, HardwareMap _hardwareMap) {
        turretMotor = _hardwareMap.get(DcMotorEx.class, motorNames.Turret);
        aimServo1 = _hardwareMap.servo.get(motorNames.AimServo1);
        aimServo2 = _hardwareMap.servo.get(motorNames.AimServo2);
        kicker = _hardwareMap.servo.get(motorNames.kickerServo);
        shooter = _hardwareMap.get(DcMotorEx.class, motorNames.Shooter);

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setTargetPosition(0);
        turretMotor.setTargetPositionTolerance(15);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

    //angle of wall is 125 degrees
    //blue team april tag ID = 20
    //red team april tag ID =24
    // moving the turret counter clockwise is +
    // moving the turret clock wise is -
    public void aim(TEAMCOLOR teamcolor) {
        int targetedId = teamcolor == TEAMCOLOR.RED ? 24 : 20;
        aiming = true;
        if(!aiming){
            return;
        }
        ArrayList<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.id == targetedId) {
                    point = detection.center;
                    if(Math.abs(detection.center.x - 640)<100){
                        target = turretMotor.getCurrentPosition();
                        aiming = false;
                    }
                }
            }
        }
    }
    public void push(){
        kicker.setPosition(1.0);
        rateLimit.reset();
    }

    public void shoot() {
        shooting = ! shooting;
        if (revved) {
            shooting = false;
        }
    }

    public void update() {
        RPM = (shooter.getVelocity() / 28) * 60;
        revved = Math.abs(shooterConstants.targetRPM - RPM) < shooterConstants.tolerance;
        if (rateLimit.hasExpired()) {
            kicker.setPosition(0.0);
        }
        if (shooting && ! revved) {
            shooter.setVelocity(shooterConstants.targetRPM * 28);
            shooter.setPower(1.0);
        } else if(!shooting && revved){
            shooter.setPower(0.0);
        }
        if (Math.abs(point.x - 640) > 100 && !turretMotor.isBusy()) {
            target = target +  100 * ((point.x > 640)?1:-1);
            turretMotor.setPower(1.0);
        }
    }
}