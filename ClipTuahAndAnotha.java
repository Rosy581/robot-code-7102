package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.teamcode.hardware.Drive;
import org.firstinspires.ftc.teamcode.hardware.Slide;
import org.firstinspires.ftc.teamcode.resources.PIDController2D;

@Autonomous

public class ClipTuahAndAnotha extends LinearOpMode {

    private CRServo claw;
    private SparkFunOTOS otos;
    private Drive robot;
    private Slide slide;
    private PIDController2D pidController;
    
    @Override

    public void runOpMode() throws InterruptedException {
        pidController = new PIDController2D(0.05, 0.05, 0.01, 0.05, 0.05, 0.1, 0.1, 0.0, 0.0, 1.8);
        double targetX, targetY;
        claw = hardwareMap.crservo.get("claw");
        slide = new Slide(hardwareMap, this);
        robot = new Drive(hardwareMap, this);

        SparkFunOTOS.Pose2D pos = robot.getPosition();
        
        telemetry.addData("xPos", pos.x);
        telemetry.addData("x",0);
        telemetry.addData("yPos", pos.y);
        telemetry.addData("y",0);
        telemetry.addData("r",0);
        
        int stage = 0;
        
        telemetry.update();
        
        waitForStart();
        while(opModeIsActive()){

            pos = robot.getPosition();
            double currentX  = pos.x;
            double currentY  = pos.y;
            double currentR  = pos.h;

            double[] outputs = pidController.calculate(currentX, currentY, currentR);
            double outputX   = outputs[0];
            double outputY   = outputs[1];
            double outputR   = outputs[2];
            telemetry.addData("xPos", pos.x);
            telemetry.addData("X",outputX);
            telemetry.addData("yPos", pos.y);
            telemetry.addData("Y",outputY);
            telemetry.addData("stage",stage);
            telemetry.addData("Heading",pos.h);
            telemetry.addData("R",outputR);
            telemetry.update();
            
            switch(stage){
                case 0:
                    //grip specimen, move to bar lvl, move backarm back
                    claw.setPower(-0.25);
                    slide.moveTo(2750);
                    robot.moveArm(1000);
                    stage++;
                    break;
                case 1:
                    //move to sub
                    pidController.setTarget(0, 31, 0);
                    stage++;
                    break;
                case 2:
                    //wait till its there
                    if(pidController.atTarget && !slide.isBusy()){
                        stage++;
                    }
                    break; 
                case 3:
                    //clip specimen onto bar
                    slide.moveTo(2250);
                    if(!slide.isBusy()){
                        stage++;
                    }
                    break;
                case 4:
                    //release the specimen
                    claw.setPower(1);
                    Thread.sleep(250);
                    stage++;
                    break;
                case 5:
                    //back up in order not to hit the frame
                    pidController.setTarget(0, 26, 0);
                    stage++;
                case 6:
                    if(pidController.atTarget){
                        stage++;
                        pidController.setSpeedScale(1.0);
                    }
                    break;
                case 7:
                    //move to the space between the sub and the samples on the ground
                    pidController.setTarget(30, 26, 0);
                    stage++;
                    break;
                case 8:
                    if(pidController.atTarget){
                        stage++;
                    }
                    break;
                case 9:
                    //prepare to push sample(s) into observation zone
                    pidController.setTarget(30, 52, 0);
                    stage++;
                    break;
                case 10:
                    if(pidController.atTarget){
                        stage++;
                    }
                    break;
                case 11:
                    //move over to the sample
                    pidController.setTarget(35, 52, 0);
                    stage++;
                    break;
                case 12:
                    if(pidController.atTarget){
                        stage++;
                    }
                    break;
                case 13:
                    //push first sample
                    pidController.setTarget(58, 4, 0);    
                    if(pidController.atTarget){
                        stage++;
                    }
                    break;
                case 14:
                    //get out so human can grab sample
                    pidController.setTarget(58, 28, 180);
                    if(pidController.atTarget){
                        stage++;
                    }
                    break;
                case 15:
                    //move slide to the right level and turn around
                    slide.moveTo(850);
                    robot.rotateTo(180);
                    stage++;
                    break;
                case 16:
                    //go to the wall SLOWLY
                    pidController.setSpeedScale(3);
                    pidController.setTarget(58, 1, 180);
                    stage++;
                case 17:
                    if(pidController.atTarget){
                        stage++;
                    }
                    break;
            }         
            robot.strafeDrive(outputX, outputY, outputR);
            Thread.sleep(12);
        }
    }    
}