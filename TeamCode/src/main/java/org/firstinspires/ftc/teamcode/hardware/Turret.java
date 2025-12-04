package org.firstinspires.ftc.teamcode.hardware;

import android.util.Size;

import com.qualcomm.hardware.rev.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.configurables.Config.shooterConstants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Turret {
    final float decimation = 2;
    public final static double gearRatio = 100.0/28;
    public boolean onTarget = false;
    public double RPM = 0;
    public boolean revved = false;
    public double shooterPower = 0;
    private double prevPos;
    private double lastTime;
    public boolean shooting = false;
    public boolean aiming = false;
    public double lastSeen = 0;
    private LinearOpMode opMode;
    public AprilTagProcessor aprilTag;
    public int targetTagID = 0;
    public Thread aimingThread;
    public DcMotor turretMotor;
    public Servo aimServo;
    private DcMotor shooter;
    public VisionPortal camView;
    public IMU imu;
    public Turret(LinearOpMode _opMode, HardwareMap _hardwareMap, String _turretRotationDriveMotor, String _turretAimServo,String _shooter,String _camera){

        turretMotor    = _hardwareMap.dcMotor.get(_turretRotationDriveMotor);
        aimServo       = _hardwareMap.servo.get(_turretAimServo);
        shooter        = _hardwareMap.dcMotor.get(_shooter);

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu            = _hardwareMap.get(IMU.class,"imu");
        Camera  camera = _hardwareMap.get(Camera.class,_camera);

        opMode    = _opMode;

        Size resolution = new Size(1280,720);

        camView = new VisionPortal.Builder()
                .setCameraResolution(resolution)
                .setCamera(camera.getCameraName())
                .build();

        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(decimation);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

    }
    public enum TEAMCOLOR {
        RED,
        BLUE
    }
    public double getRotation(){
            return (turretMotor.getCurrentPosition()/28.0)*(gearRatio)*180+imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    //angle of wall is 125 degrees
    //blue team april tag ID = 20
    //red team april tag ID = 24

    private class _aim implements Runnable{
        TEAMCOLOR color;
        _aim(TEAMCOLOR _color){
            this.color = _color;
        }

        @Override
        public void run() {
            targetTagID = 0;
            if (color == TEAMCOLOR.RED) {
                targetTagID = 24;
            } else if (color == TEAMCOLOR.BLUE){
                targetTagID = 20;
            }

            while(aiming) {
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                if(!currentDetections.isEmpty()) {
                    for (AprilTagDetection detection : currentDetections) {
                        if (detection.metadata != null) {
                            if ((targetTagID < 0) || (detection.id == targetTagID)) {
                                turretMotor.setPower(0.0);
                                lastSeen = getRotation();
                                onTarget = true;
                                aiming = false;
                                break;
                            }
                        } else {
                            onTarget = false;
                        }
                    }
                } else {
                    onTarget = false;
                    turretMotor.setTargetPosition((int) Math.round(28*(lastSeen-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES))/180/gearRatio));
                    if(!turretMotor.isBusy()){
                        turretMotor.setTargetPosition(turretMotor.getTargetPosition()+50);
                    }
                    //Runs to (hopefully) last seen location
                    turretMotor.setPower(0.75);
                }
            }
        }
    }

    public void aim(TEAMCOLOR color){
        if(aiming){return;}
        aiming = true;
        aimingThread = new Thread(new _aim(color));
    }

    public void shoot(TEAMCOLOR color){
        if(revved & onTarget){
            //move blocker out of the way
            //shoot
            shooting = false;
        } else {
            aim(color);
        }
    }

    public void updateRPM(){
        RPM = Math.abs((shooter.getCurrentPosition())-prevPos)/1680*(1000/(lastTime-opMode.getRuntime()));
        prevPos = shooter.getCurrentPosition();
        lastTime = opMode.getRuntime();
        revved = (Math.abs(shooterConstants.targetRPM - RPM) > shooterConstants.tolerance);
        if(shooting && !revved) {
            shooterPower = shooterConstants.Kp * (shooterConstants.targetRPM - RPM) + shooterConstants.Kv * (shooterConstants.targetRPM);
            shooter.setPower(shooterPower);
        }
    }
}