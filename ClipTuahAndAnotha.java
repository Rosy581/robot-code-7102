package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
        pidController = new PIDController2D(0.04, 0.0, 0.0, 0.04, 0.0, 0.0, 0.1, 0.0, 0.0, 4.0);
        double targetX, targetY;
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        claw = hardwareMap.crservo.get("claw");
        slide = new Slide(hardwareMap, this);
        robot = new Drive(hardwareMap, this);
        
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);
        otos.setOffset(new SparkFunOTOS.Pose2D (0.0, -8.5, 0.0));
        otos.setLinearScalar(1.03125);
        otos.setAngularScalar(1.00416666667);
        otos.calibrateImu();
        otos.resetTracking();
        otos.setPosition(new SparkFunOTOS.Pose2D(0,0,0));
        
        SparkFunOTOS.Pose2D pos = otos.getPosition();
        
        telemetry.addData("xPos", pos.x);
        telemetry.addData("x",0);
        telemetry.addData("yPos", pos.y);
        telemetry.addData("y",0);
        telemetry.addData("r",0);
        
        int stage = 0;
        
        telemetry.update();
        
        waitForStart();
        while(opModeIsActive()){

            pos = otos.getPosition();
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
                    pidController.setSetpoints(0, 30, 0);
                    if(pidController.atTarget){
                        stage++;
                    }
                    break;
                case 2:
                    //clip specimen onto bar
                    slide.moveTo(2250);
                    if(!slide.isBusy()){
                        stage++;
                    }
                    break;
                case 3:
                    //release the specimen
                    claw.setPower(1);
                    Thread.sleep(250);
                    stage++;
                    break;
                case 4:
                    //back up in order not to hit the frame
                    pidController.setTarget(0, 28, 0);
                    if(pidController.atTarget){
                        stage++;
                    }
                    break;
                case 5:
                    //move to the space between the sub and the samples on the ground
                    pidController.setTarget(30, 28, 0);
                    if(pidController.atTarget){
                        stage++;
                    }
                    break;
                case 6:
                    // prepare to push samples into observation zone
                    pidController.setTarget(30, 52, 0);
                    if(pidController.atTarget){
                        stage++;
                    }
                    break;
                case 7:
            }            
            robot.strafeDrive(outputX, outputY, outputR);

            Thread.sleep(12);
        }
    }    
}