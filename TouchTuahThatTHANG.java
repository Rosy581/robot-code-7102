package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.logging.Logger;
import java.util.logging.Level;
import java.io.IOException;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.hardware.Slide;
import org.firstinspires.ftc.teamcode.hardware.Drive;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import java.util.concurrent.TimeUnit;
import java.util.logging.FileHandler;
import java.util.logging.SimpleFormatter;

@Autonomous(name = "Clip Tuah that THANG (touching edition)", group = "Robot")

public class TouchTuahThatTHANG extends LinearOpMode {
    private static final Logger logger = Logger.getLogger(TouchTuahThatTHANG.class.getName());
    private ElapsedTime runtime = new ElapsedTime();
    private CRServo claw;
    private DcMotor slide;
    private Drive robot;
    Deadline rateLimit = new Deadline(250, TimeUnit.MILLISECONDS);

    static final double DRIVE_SPEED = 0.5;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        try{
            FileHandler fh = new FileHandler("/logs/LOG.txt",true);
            SimpleFormatter format = new SimpleFormatter();
            fh.setFormatter(format);
            logger.addHandler(fh);
            logger.setLevel(Level.ALL);
        } catch (IOException e){
            e.printStackTrace();
        }

        claw = hardwareMap.crservo.get("claw");
        slide = hardwareMap.dcMotor.get("slide");
        robot = new Drive(hardwareMap, this);

        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();

        telemetry.addData("Yaw (Z)", "%.1f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.update();
        String state = "Begin";
        waitForStart();
        while (opModeIsActive() && state != "finished") {
            switch(state){
                case "Begin":  
                    logger.info("Started");
                    claw.setPower(-0.25);
                    slide.setTargetPosition(2750);
                    robot.moveArm(1000);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(1); 
                    state = "step 1";
                    break;
                case "step 1":
                    robot.encoderDrive(0.35, 12.0, 12.0);
                    state = "step 2";
                    break;
                case "step 2":
                    if(!slide.isBusy()){
                        robot.encoderDrive(0.5, -0.75,-0.75); 
                        state = "step 3";
                    }
                    break;
                case "step 3":
                    slide.setTargetPosition(2250);
                    state = "step 4";
                    rateLimit.reset();
                    break;
                case "step 4":
                    if(!slide.isBusy() && rateLimit.hasExpired()){
                        robot.backArm1.setPower(0);
                        robot.backArm2.setPower(0);
                        state = "step 5";
                    }
                    break;
                case "step 5":
                    claw.setPower(1);
                    robot.encoderDrive(1,-2.0,-2.0);
                    state = "step 6";
                    break;
                case "step 6":
                    robot.encoderDrive(0.75, -2.3, -2.3);
                    state = "step 7";
                    break;
                case "step 7":
                    robot.rotateTo(90,-0.25);
                    robot.encoderDrive(1,15);
                    robot.setPower(.25);   
                    state = "step 8";
                    break;
                case "step 8":  
                    if(robot.isTouched()){
                        robot.setPower(0);
                        robot.encoderDrive(0.5,-3);
                        robot.rotateTo(175,-0.5); 
                        Thread.sleep(100);
                        state = "step 9"; 
                    }
                    break;
                case "step 9":
                    robot.setPower(0.25);
                    state = "step 10";
                    break;
                case "step 10":
                    if(robot.isTouched()){
                        state = "step 11";
                    }
                    break;
                case "step 11":
                    robot.encoderDrive(0.75, -6, -6);
                    slide.setTargetPosition(850);
                    Thread.sleep(750);
                    state = "step 12";
                    break;
                case "step 12": 
                    robot.encoderDrive(0.75, 6); 
                    claw.setPower(-0.25);
                    Thread.sleep(300);
                    state = "step 13";
                    break;
                case "step 13":
                    slide.setTargetPosition(2750);
                    robot.encoderDrive(1,-4.5);
                    state = "step 14";
                    break; 
                case "step 14":
                    robot.moveRight(0.75,12);
                    state = "step 15";
                     break;
                case "step 15":
                    if(!slide.isBusy()){
                        state = "step 16";
                    }
                    break;
                case "step 16":
                    robot.rotateTo(0,0.5);
                    robot.moveLeft(0.5,12);
                    robot.encoderDrive(0.25, 5);
                    robot.encoderDrive(0.5, -0.75);
                    state = "step 17";
                    break;
                case "step 17":
                    slide.setTargetPosition(2250);
                    state = "step 18";
                    rateLimit.reset();
                    break;
                case "step 18":
                    if(!slide.isBusy() && rateLimit.hasExpired()){
                        state = "step 19";
                    }
                    break;
                case "step 19":
                    claw.setPower(1);
                    robot.encoderDrive(1,-2.0);
                    state = "step 20";
                    break;
                case "step 20":
                    robot.moveArm(0);
                    robot.encoderDrive(0.5,-12);
                    claw.setPower(0);
                    slide.setTargetPosition(0);
                    state = "step 21";
                case "step 21":
                    robot.moveRight(100, 3000);
                    state = "finished";
                    break;
                }
            telemetry.addData("heading", robot.getHeading());
            telemetry.addData("Step", state);
            telemetry.update();
        } 
        
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1250);
    }   
}
 