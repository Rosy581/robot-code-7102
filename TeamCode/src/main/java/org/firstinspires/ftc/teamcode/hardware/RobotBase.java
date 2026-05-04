package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Config.partNames;
import org.firstinspires.ftc.teamcode.hardware.Config.partDirection;
import org.opencv.core.Point;

public class RobotBase {
    public OPMODETYPE opModeType = OPMODETYPE.TEST;
    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor backRight;
    public DcMotor backLeft;
    public DcMotor intake;
    private GoBildaPinpointDriver odo;
    public static Point odoOffset = new Point(0.0,0.0);
    public static final Gamepad.LedEffect RedLights = new Gamepad.LedEffect.Builder().addStep(1.0,0,0,1000).setRepeating(true).build();
    public static final Gamepad.LedEffect BlueLights = new Gamepad.LedEffect.Builder().addStep(0.0,0.0,1.0,1000).setRepeating(true).build();

    public RobotBase(@NonNull HardwareMap _hardwareMap, OPMODETYPE _opModeType) {
        frontRight = _hardwareMap.dcMotor.get(partNames.FrontRight);
        frontLeft  = _hardwareMap.dcMotor.get(partNames.FrontLeft);
        backRight  = _hardwareMap.dcMotor.get(partNames.BackRight);
        backLeft   = _hardwareMap.dcMotor.get(partNames.BackLeft);


        frontRight .setDirection(DIRECTION.convertMotor(partDirection.FrontRight));
        frontLeft  .setDirection(DIRECTION.convertMotor(partDirection.FrontLeft));
        backRight  .setDirection(DIRECTION.convertMotor(partDirection.BackRight));
        backLeft   .setDirection(DIRECTION.convertMotor(partDirection.BackLeft));

        intake     = _hardwareMap.dcMotor.get(partNames.Intake);

        odo        = _hardwareMap.get(GoBildaPinpointDriver.class, partNames.Odometry);

        opModeType = _opModeType;
        switch (opModeType){
            case TELE:
            case AUTO:
            case TEST:
                break;
            default:
                throw new IllegalArgumentException("NO OPMODE TYPE SPECIFIED");
        }
    }
    public enum DIRECTION{
        FORWARD,
        BACKWARD;
        public static GoBildaPinpointDriver.EncoderDirection convertOdo (DIRECTION direction){
            return direction == FORWARD?GoBildaPinpointDriver.EncoderDirection.FORWARD:GoBildaPinpointDriver.EncoderDirection.REVERSED;
        }
        public static DcMotorSimple.Direction convertMotor (DIRECTION direction){
            return direction == FORWARD?DcMotorSimple.Direction.FORWARD:DcMotorSimple.Direction.REVERSE;
        }
    }
    public enum TEAMCOLOR {
        RED,
        BLUE
    }
    public enum OPMODETYPE{
        TELE,
        AUTO,
        TEST
    }
    public void update(TEAMCOLOR teamcolor){
        odo.update();
    }
    public void configOdo() {
        odo.setOffsets(odoOffset.x,odoOffset.y,DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(DIRECTION.convertOdo(partDirection.odoX), DIRECTION.convertOdo(partDirection.odoY));
        odo.resetPosAndIMU();
    }
    public void configureDriveTrainZeroPower(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        frontLeft.setZeroPowerBehavior(zeroPowerBehavior);
        frontRight.setZeroPowerBehavior(zeroPowerBehavior);
        backLeft.setZeroPowerBehavior(zeroPowerBehavior);
        backRight.setZeroPowerBehavior(zeroPowerBehavior);
    }
    public void mecanumDrive(double x, double y, double rx) {
        double denominator     = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rx), 1);
        double frontRightPower = (y - x - rx) / denominator;
        double frontLeftPower  = (y + x + rx) / denominator;
        double backRightPower  = (y + x - rx) / denominator;
        double backLeftPower   = (y - x + rx) / denominator;

        frontRight.setPower(frontRightPower);
        frontLeft.setPower(frontLeftPower);
        backRight.setPower(backRightPower);
        backLeft.setPower(backLeftPower);
    }
}