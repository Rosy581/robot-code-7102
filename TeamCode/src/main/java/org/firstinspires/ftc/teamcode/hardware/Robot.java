package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.configurables.Config.shooterConstants;
import org.firstinspires.ftc.teamcode.configurables.Config.partNames;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Robot {
    public double angle = 0;
    public boolean shooting = false;
    public boolean aiming = false;
    public boolean revved = false;
    public boolean turning = false;
    public double targetRPM = shooterConstants.targetRPMclose;
    public double RPM = 0;
    public DcMotorEx turretMotor;
    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor backRight;
    public DcMotor backLeft;
    private DcMotor intake;
    public DcMotor feeder;
    private DcMotorEx shooter;
    private double intakeSpeed = 0;
    private GoBildaPinpointDriver odo;

    public final static double turretEncoderResolution = 537.7;
    private final static double gearRatio = 35.0 / 110.0;
    private final static Pose2D RedTarget = new Pose2D(DistanceUnit.INCH, 144, 144, AngleUnit.DEGREES, 0);
    private final static Pose2D BlueTarget = new Pose2D(DistanceUnit.INCH, 144, 3, AngleUnit.DEGREES, 0);
    public final static Pose2D RedCorner  = new Pose2D(DistanceUnit.INCH, 6.5, 8.5, AngleUnit.DEGREES, 0);
    public final static Pose2D BlueCorner = new Pose2D(DistanceUnit.INCH, 137.5,8.5 , AngleUnit.DEGREES, 0);

    public Robot(@NonNull HardwareMap _hardwareMap) {
        frontRight = _hardwareMap.dcMotor.get(partNames.FrontRight);
        frontLeft = _hardwareMap.dcMotor.get(partNames.FrontLeft);
        backRight = _hardwareMap.dcMotor.get(partNames.BackRight);
        backLeft = _hardwareMap.dcMotor.get(partNames.BackLeft);
        turretMotor = _hardwareMap.get(DcMotorEx.class, partNames.Turret);
        shooter = _hardwareMap.get(DcMotorEx.class, partNames.Shooter);
        intake = _hardwareMap.dcMotor.get(partNames.Intake);
        feeder = _hardwareMap.dcMotor.get(partNames.Feeder);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setPower(1);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        feeder.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        odo = _hardwareMap.get(GoBildaPinpointDriver.class, partNames.Odometry);
    }
    public enum TEAMCOLOR {
        RED,
        BLUE
    }

    public void configOdo() {
        odo.setOffsets(0, 6.5, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();
    }

    public void toggleShootingDistance(){
        targetRPM = targetRPM == shooterConstants.targetRPMclose ? shooterConstants.targetRPMfar : shooterConstants.targetRPMclose;
    }
    public void toggleAim(){
        aiming = !aiming;
    }

    public void shoot() {
        shooting = ! shooting;
    }

    public void update(TEAMCOLOR teamColor) {
        odo.update();

        RPM = (shooter.getVelocity() / 28) * 60;
        revved = Math.abs(targetRPM - RPM) < shooterConstants.tolerance;

        turning = turretMotor.isBusy();

        if (shooting) {
            shooter.setVelocity((targetRPM / 60) * 28);
        } else {
            shooter.setVelocity(0.0);
        }

        if(revved && shooting){
            feeder.setPower(1);
        } else {
            feeder.setPower(0);
        }

        if (aiming) {
            angle = aimAtPos(teamColor);
        }
    }

    public double encoderToRotation(int encoderPos) {
        return (((encoderPos/turretEncoderResolution)*gearRatio) * 360) + odo.getHeading(AngleUnit.DEGREES);
    }
    public int rotationToEncoder(double angle) {
        return (int) (((angle-getHeading())/(360*gearRatio))*turretEncoderResolution);
    }

    public double getTurretRotation() {
        return encoderToRotation(turretMotor.getCurrentPosition());
    }
    public double aimAtPos(Pose2D target) {
        Pose2D pos = odo.getPosition();
        double x = Math.abs((target.getX(DistanceUnit.INCH) - pos.getX(DistanceUnit.INCH)));
        double y = Math.abs((target.getY(DistanceUnit.INCH) - pos.getY(DistanceUnit.INCH)));
        double angle = Math.toDegrees(Math.atan(x/ y)) * (pos.getX(DistanceUnit.INCH) < x ? -1 : 1) ;
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

    public void toggleIntake(){
        intakeSpeed = (intakeSpeed == 1 ? 0 : 1);
        intake.setPower(intakeSpeed);
    }
    public void setPosition(Pose2D position){
        odo.setPosition(position);
    }
    public double getHeading(AngleUnit unit){
        return odo.getHeading(unit);
    }
    public double getHeading(){
        return getHeading(AngleUnit.DEGREES);
    }
    public Pose2D getPos(DistanceUnit unit, AngleUnit angle){
        return new Pose2D(unit,odo.getPosX(unit),odo.getPosY(unit),angle,odo.getHeading(angle));
    }
    public Pose2D getPos(){
        return getPos(DistanceUnit.INCH,AngleUnit.DEGREES);
    }
    public void stop(){
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        shooter.setPower(0);
        intake.setPower(0);
        turretMotor.setPower(0);
        feeder.setPower(0);
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
    public void fieldCentricMecanumDrive(double x, double y, double rx){
        double botHeading = odo.getHeading(AngleUnit.RADIANS);
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontRight.setPower(frontRightPower);
        frontLeft.setPower(frontLeftPower);
        backRight.setPower(backRightPower);
        backLeft.setPower(backLeftPower);
    }
}