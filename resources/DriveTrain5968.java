// package org.firstinspires.ftc.teamcode.Hardware;
// 
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import com.qualcomm.robotcore.hardware.HardwareMap;
// import com.qualcomm.robotcore.hardware.IMU;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
// 
// public class DriveTrain {
//     private LinearOpMode opmode;
//     private IMU imu;
//     private DcMotor backLeftMotor;
//     private DcMotor backRightMotor;
//     private DcMotor frontLeftMotor;
//     private DcMotor frontRightMotor;
// 
//     private int eventsPerMotorRotation = 28; 
//     private double wheelMotorRatio = 19.2;
//     private double eventsPerWheelRotation = eventsPerMotorRotation * wheelMotorRatio;
//     private double inchesPerWheelRotation = 12.3;
//     private double desiredHeadingDegrees = 0;
//     private double currentrx = 0;
//     private double currentRotation = 0;
// 
//     public void init(LinearOpMode opmode, HardwareMap hwMap) {
//         this.opmode = opmode;
//         backLeftMotor = hwMap.get(DcMotor.class, "backLeftMotor");
//         backRightMotor = hwMap.get(DcMotor.class, "backRightMotor");
//         frontLeftMotor = hwMap.get(DcMotor.class, "frontLeftMotor");
//         frontRightMotor = hwMap.get(DcMotor.class, "frontRightMotor");
//     
//         frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//         backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//         frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//     
//         imu = hwMap.get(IMU.class, "imu");
//         IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                 RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                 RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
//         imu.initialize(parameters);
//         ResetForwardPosition();
//     }
//     
//     public void ResetDistanceCounters() {
//         backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         //backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//         //backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);        
//         backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//         backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);        
//     }
// 
//     public double DistanceTravelled() {
//         double averageEventsRecorded = (backLeftMotor.getCurrentPosition() + backRightMotor.getCurrentPosition())/2;
//         double wheelRotations = averageEventsRecorded / eventsPerWheelRotation;
//         return wheelRotations * inchesPerWheelRotation;
//     }
// 
//     public double RotationNeededDegrees(){
//         double botHeadingDegrees = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//         double botRotationNeeded = desiredHeadingDegrees - botHeadingDegrees;
//         if (botRotationNeeded > 180) {botRotationNeeded = botRotationNeeded - 360;}
//         if (botRotationNeeded < -180) {botRotationNeeded = botRotationNeeded + 360;}
//         return botRotationNeeded;
//     }
//     
//     public void Drive(double sideways, double forward) {
//         Drive(sideways, forward, 0, false);
//     }
//     
//     public void Drive(double sideways, double forward, double rotation, boolean fieldCentric) {
//         double rx = rotation;
//         if (rotation != (double)0) {
//             currentRotation = Math.abs(rotation);
//         } else {
//             double botRotationNeeded = RotationNeededDegrees();
//             if (botRotationNeeded != (double)0) {
//                 double botPitch = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
//                 if (botPitch <= 5) {
//                     rx = Math.min(1, (double)(Math.abs(botRotationNeeded)) / 90);
//                     rx = Math.min(rx, Math.abs(currentrx) + 0.1);
//                     if ((double)(Math.abs(botRotationNeeded)) >= 1) {rx = Math.max(rx, 0.1);}
//                     rx = Math.signum(botRotationNeeded) * -rx;
//                 }
//             }
//         }
//         currentrx = rx;
//         double frontLeftPower = 0;
//         double backLeftPower = 0;
//         double frontRightPower = 0;
//         double backRightPower = 0;
//         if (fieldCentric == true) {
//             double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//             // Rotate the movement direction counter to the bot's rotation
//             double rotX = sideways * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
//             double rotY = sideways * Math.sin(-botHeading) + forward * Math.cos(-botHeading);
// 
//             rotX = rotX * 1.1;  // Counteract imperfect strafing
// 
//             // Denominator is the largest motor power (absolute value) or 1
//             // This ensures all the powers maintain the same ratio,
//             // but only if at least one is out of the range [-1, 1]
//             double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
//             frontLeftPower = (rotY + rotX + rx) / denominator;
//             backLeftPower = (rotY - rotX + rx) / denominator;
//             frontRightPower = (rotY - rotX - rx) / denominator;
//             backRightPower = (rotY + rotX - rx) / denominator;
//         } else {
//             // Denominator is the largest motor power (absolute value) or 1
//             // This ensures all the powers maintain the same ratio,
//             // but only if at least one is out of the range [-1, 1]
//             double denominator = Math.max(Math.abs(forward) + Math.abs(sideways) + Math.abs(rx), 1);
//             frontLeftPower = (forward + sideways + rx) / denominator;
//             backLeftPower = (forward - sideways + rx) / denominator;
//             frontRightPower = (forward - sideways - rx) / denominator;
//             backRightPower = (forward + sideways - rx) / denominator;
//         }
//         frontLeftMotor.setPower(frontLeftPower);
//         frontRightMotor.setPower(frontRightPower);
//         backRightMotor.setPower(backRightPower);
//         backLeftMotor.setPower(backLeftPower);
//         if (currentRotation >= 0) {
//             currentRotation -= 0.1;
//             SetHeading(CurrentHeading());
//         }
//     }
// 
//     public void ResetForwardPosition() {
//         imu.resetYaw();
//         desiredHeadingDegrees = 0;
//     }
//     
//     public double CurrentHeading() {
//         return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//     }
// 
//     public double DesiredHeading() {
//         return desiredHeadingDegrees;
//     }
// 
//     public void StopBot() {
//         frontLeftMotor.setPower(0);
//         backLeftMotor.setPower(0);
//         frontRightMotor.setPower(0);
//         backRightMotor.setPower(0);
//     }
// 
//     public void SetHeading(double desiredHeadingDegrees){
//         this.desiredHeadingDegrees = desiredHeadingDegrees;
//     }
// 
//     public void TurnToHeading(double desiredHeadingDegrees){
//         SetHeading(desiredHeadingDegrees);
//         while (Math.abs(RotationNeededDegrees()) >= 1 && opmode.opModeIsActive()) {
//             Drive(0, 0);
//         }
//         StopBot();
//     }
// 
//     private double DesiredForwardPower(double remainingRotations, double maxPower) {
//         if (remainingRotations <= 0) {
//             return 0;
//         }
//         double desiredPower = Math.min(maxPower, remainingRotations / 2);
//         desiredPower = Math.max(desiredPower, 0.1);
//         return Math.round(desiredPower * 20.0) / 20.0;
//     }
// 
//     public void DriveStraight(double inchesToMove, double MaxPower) {
//         ResetDistanceCounters();
//         double neededWheelRotations = inchesToMove / inchesPerWheelRotation;
//         double eventsNeededPerWheel = neededWheelRotations * eventsPerWheelRotation;
//         double remainingRotations = Math.abs(neededWheelRotations);
//         double desiredPower = MaxPower;
//         double currentPower = 0;
//         while (remainingRotations > 0 && opmode.opModeIsActive()) {
//             double averageEventsRecorded = (backLeftMotor.getCurrentPosition() + backRightMotor.getCurrentPosition())/2;
//             double currentWheelRotations = averageEventsRecorded / eventsPerWheelRotation;
//             remainingRotations = Math.abs(neededWheelRotations) - Math.abs(currentWheelRotations);
//             desiredPower = DesiredForwardPower(remainingRotations, MaxPower);
//             if (desiredPower > currentPower) {
//                 desiredPower = Math.min(desiredPower, currentPower + 0.05);
//                 currentPower = Math.abs(desiredPower);
//             }
//             desiredPower = Math.signum(inchesToMove) * desiredPower;
//             Drive(0, desiredPower);
//         }
//         StopBot();
//     }
// }
// 
