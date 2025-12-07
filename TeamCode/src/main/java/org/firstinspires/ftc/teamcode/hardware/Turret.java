package org.firstinspires.ftc.teamcode.hardware;

import android.util.Size;

import com.qualcomm.hardware.rev.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.configurables.Config.shooterConstants;
import org.firstinspires.ftc.teamcode.configurables.Config.motorNames;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Turret {
    public boolean onTarget = false;
    public boolean one = false;
    public boolean two = false;
    public boolean three = false;
    public boolean four = false;
    public boolean five = false;
    public int target = 0;
    public boolean shooting = false;
    public boolean aiming = false;
    public boolean revved = false;
    public VisionPortal camView;
    public final static double encoderResolution = 537.7;
    public double RPM = 0;
    public double shooterPower = 0;
    public Thread aimingThread;
    public DcMotor turretMotor;
    public Servo aimServo1;
    public Servo aimServo2;
    public Servo blocker;
    private final static float decimation = 2;
    private final static double gearRatio = 198/48.0;
    private double prevPos;
    private double lastTime;
    private int lastSeen = 0;
    private LinearOpMode opMode;
    private AprilTagProcessor aprilTag;
    private DcMotor shooter;
    private IMU imu;
    private final static int minPos = (int) Math.round((-0.5)*gearRatio*encoderResolution);
    private final static int maxPos = -minPos;
    public Turret(LinearOpMode _opMode, HardwareMap _hardwareMap){

        turretMotor     = _hardwareMap.dcMotor.get(motorNames.Turret);
        aimServo1       = _hardwareMap.servo.get(motorNames.AimServo1);
        aimServo2       = _hardwareMap.servo.get(motorNames.AimServo2);
        blocker         = _hardwareMap.servo.get(motorNames.BlockerServo);
        shooter         = _hardwareMap.dcMotor.get(motorNames.Shooter);

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu            = _hardwareMap.get(IMU.class,"imu");

        opMode    = _opMode;

        Size resolution = new Size(1280,720);

        aprilTag = new AprilTagProcessor.Builder().build();
        camView = new VisionPortal.Builder()
                .setCameraResolution(resolution)
                .setCamera(_hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        aprilTag.setDecimation(decimation);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

    }
    public enum TEAMCOLOR {
        RED,
        BLUE
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
            int last = 0;
            int targetTagID = 0;
            if (color == TEAMCOLOR.RED) {
                targetTagID = 24;
            } else if (color == TEAMCOLOR.BLUE){
                targetTagID = 20;
            }
            while(aiming) {
                one = true;
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                if(!currentDetections.isEmpty()) {
                    two = true;
                    for (AprilTagDetection detection : currentDetections) {
                        three = true;
                        if (detection.metadata != null) {
                            if (detection.id == targetTagID) {
                                turretMotor.setPower(0.0);
                                lastSeen = turretMotor.getCurrentPosition();
                                onTarget = true;
                                aiming = false;
                                break;
                            }
                        } else {
                            onTarget = false;
                        }
                    }
                } else {
                    if(!turretMotor.isBusy()) {
                        four = true;
                        onTarget = false;
                        if (turretMotor.getCurrentPosition() == target) {
                            five = true;
                            target = target + (((last>target)?1:-1)*(int) (encoderResolution));
                        } else {
                            target = lastSeen;
                        }
                        last = turretMotor.getCurrentPosition();
//                        if ((minPos > target) || (target > maxPos)){
//                            target = target % 180 *-1;
//                        }
                        turretMotor.setTargetPosition(target);
                        turretMotor.setPower(0.75);
                    }
                }
            }
        }
    }

    public void aim(TEAMCOLOR color){
        if(aiming){return;}
        aiming = true;
        aimingThread = new Thread(new _aim(color));
        aimingThread.start();
    }

    public void shoot(){
        if(!shooting){
            shooting = true;
        }
        if(revved & onTarget){
            //move blocker out of the way
            //shoot
            shooting = false;
        }
    }

    public void updateRPM(){
        RPM = Math.abs((shooter.getCurrentPosition())-prevPos)/1680*(1000/(lastTime-opMode.getRuntime()));
        prevPos = shooter.getCurrentPosition();
        lastTime = opMode.getRuntime();
        revved = (shooterConstants.targetRPM - RPM) < shooterConstants.tolerance;
        if(shooting && !revved) {
            shooterPower = Math.abs(shooterConstants.Kp * (shooterConstants.targetRPM - RPM) + shooterConstants.Kv * (shooterConstants.targetRPM));
            shooter.setPower(shooterPower);
        }
    }
}