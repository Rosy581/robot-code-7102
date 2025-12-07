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
import org.firstinspires.ftc.vision.VisionPortal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.configurables.Config.shooterConstants;
import org.firstinspires.ftc.teamcode.configurables.Config.motorNames;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Robot {
    public boolean onTarget = false;
    public static int target = 0;
    public boolean shooting = false;
    public boolean aiming = false;
    public boolean revved = false;
    public VisionPortal camView;
    public final static double encoderResolution = 145.1;
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
    private final static int minPos = 1060;
    private final static int maxPos = - minPos;
    private final static Pose2D RedPos = new Pose2D(DistanceUnit.INCH, 9, 135, AngleUnit.DEGREES, 0);
    private final static Pose2D BluePos = new Pose2D(DistanceUnit.INCH, 135, 135, AngleUnit.DEGREES, 0);

    public Robot(LinearOpMode _opMode, HardwareMap _hardwareMap) {
        turretMotor = _hardwareMap.get(DcMotorEx.class, motorNames.Turret);
        aimServo1 = _hardwareMap.servo.get(motorNames.AimServo1);
        aimServo2 = _hardwareMap.servo.get(motorNames.AimServo2);
        //kicker         = _hardwareMap.servo.get(motorNames.kickerServo);
        shooter = _hardwareMap.get(DcMotorEx.class, motorNames.Shooter);

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
    }

    public enum TEAMCOLOR {
        RED,
        BLUE
    }

    //angle of wall is 125 degrees
    //blue team april tag ID = 20
    public void aim(TEAMCOLOR teamcolor) {
        Pose2D target = teamcolor == TEAMCOLOR.RED?RedPos:BluePos;
        Pose2D position = odo.getPosition();
        double Angle = Math.atan(Math.abs(position.getX(DistanceUnit.INCH)-target.getX(DistanceUnit.INCH))/Math.abs(position.getY(DistanceUnit.INCH)-target.getX(DistanceUnit.INCH)));
        double targetPosition =
    }

    public void shoot() {
        if (! shooting) {
            shooting = true;
        }
        if (revved & onTarget) {

            shooting = false;
        }
    }

    public void update() {
        RPM = shooter.getVelocity() / 28;
        revved = (shooterConstants.targetRPM - RPM) < shooterConstants.tolerance;
        if (shooting && ! revved) {
            shooter.setVelocity(shooterConstants.targetRPM * 28);
            shooter.setPower(1.0);
        } else {
            shooter.setPower(0.0);
        }
    }
}