package org.firstinspires.ftc.teamcode.hardware;

import android.util.Size;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.configurables.Config.shooterConstants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Turret {
    static HardwareMap hardwareMap;
    static final float decimation = 2;
    public static boolean onTarget = false;
    public static double RPM = 0;
    public double shooterPower = 0;
    private double prevPos;
    private double lastTime;
    private static DcMotor shooter;
    public boolean shooting = false;
    private static LinearOpMode tele;
    public static AprilTagProcessor aprilTag;
    public static int targetTag = 0;
    private static AprilTagDetection targetedTag = null;
    private static TelemetryManager telemetry;
    public Turret(LinearOpMode _opMode, HardwareMap _hardwareMap, String _turretRotationDriveMotor, String _turretAimServo, String _camera, TelemetryManager _telemetry){
        this.hardwareMap = _hardwareMap;

        DcMotor turretMotor  = hardwareMap.dcMotor.get(_turretRotationDriveMotor);
        Servo   aimServo     = hardwareMap.servo.get(_turretAimServo);
        Camera  camera       = hardwareMap.get(Camera.class,_camera);
        this.telemetry = _telemetry;
        tele = _opMode;

        Size resolution = new Size(1280,720);

        VisionPortal camView = new VisionPortal.Builder()
                .setCameraResolution(resolution)
                .setCamera(camera.getCameraName())
                .build();

        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(decimation);
    }
    public enum TEAMCOLOR {
        RED,
        BLUE
    }
    public static double getRotation(){
        double rotation = 0;
        return rotation;
    }
    public static void aim(TEAMCOLOR color){
        //angle of wall is 125 degrees
        //blue team ID = 20
        //red team ID = 24
        if (color == TEAMCOLOR.RED) {
            targetTag = 24;
        } else if (color == TEAMCOLOR.BLUE){
            targetTag = 20;
        }
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((targetTag < 0) || (detection.id == targetTag)) {
                    // Yes, we want to use this tag.
                    onTarget = true;
                    targetedTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData( "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Tag ID %d is not in TagLibrary", detection.id);
            }
        }
    }
    public void shoot(){
        if(shooting){
            if((Math.abs(shooterConstants.targetRPM - RPM) > shooterConstants.tolerance)) {
                shooterPower = shooterConstants.Kp * (shooterConstants.targetRPM - RPM) + shooterConstants.Kv * (shooterConstants.targetRPM);
                shooter.setPower(shooterPower);
            } else if(onTarget){
                //move blocker out of the way
                shooting = false;
            }
        }
    }

    public void getRPM(){
        RPM = Math.abs((shooter.getCurrentPosition())-prevPos)/1680*(1000/(lastTime-tele.getRuntime()));
        prevPos = shooter.getCurrentPosition();
        lastTime = tele.getRuntime();
    }
}