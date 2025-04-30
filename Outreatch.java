// package org.firstinspires.ftc.teamcode;
// 
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.Gamepad;
// 
// 
// @com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Outreatch", group = "")
// public class Outreatch extends LinearOpMode {
// 
//     private DcMotor motor1;
//     private DcMotor motor2;
//     private double motorPower;
//     private double leftMotorPower;
//     private double rightMotorPower;
//     Gamepad previousGamepad1 = new Gamepad();
//     
//     @Override
//     public void runOpMode() {
//          
//         // Initialize the motors
//         motor1 = hardwareMap.get(DcMotor.class, "motor1");
//         motor2 = hardwareMap.get(DcMotor.class, "motor2");
// 
//         // Set the motors to run without encoders (optional, depending on your needs)
//         motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//         motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//         motor2.setDirection(DcMotor.Direction.REVERSE);
// 
//         // Wait for the start button to be pressed
//         waitForStart();
// 
//         while (opModeIsActive()) {
//             double forward = gamepad1.left_stick_y;  // Forward/backward motion
//             double strafe = gamepad1.left_stick_x;   // Left/right motion
//             double rotate = 0.8 * gamepad1.right_stick_x;  // Rotation (clockwise/counterclockwise)
// 
// // Motor power adjustments for continuous 360-degree drive
//             leftMotorPower = (forward - rotate - 0.5 * strafe);
//             rightMotorPower = (forward + rotate + 0.5 * strafe);
//             
//             // Control the motors based on gamepad input
//             // if (gamepad1.left_trigger > 0) {
//             //     // Move forward
//             //     motorPower = left_trigger;
//             // } else if (gamepad1.right_trigger < 0) {
//             //     // Move backward
//             //     motorPower = right_trigger;
//             // } else {
//             //     // Stop the motors
//             //     motorPower = 0;
//             // }
//             
//             motor1.setPower(leftMotorPower);
//             motor2.setPower(rightMotorPower);
// 
//             // Add telemetry for debugging
//             telemetry.addData("Motor 1 Power", leftMotorPower);
//             telemetry.addData("Motor 2 Power", rightMotorPower);
//             telemetry.update();
//         }
//     }
// }
// 