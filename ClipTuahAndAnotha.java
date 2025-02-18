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
import org.firstinspires.ftc.teamcode.Datalogger;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Autonomous

public class ClipTuahAndAnotha extends LinearOpMode {

    private CRServo claw;
    private SparkFunOTOS otos;
    private Drive robot;
    private Slide slide;
    private VoltageSensor battery;
    private PIDController2D pidController;
    public Datalogger.GenericField opModeStatus = new Datalogger.GenericField("OpModeStatus");
    public Datalogger.GenericField stage        = new Datalogger.GenericField("Stage");
    public Datalogger.GenericField yaw          = new Datalogger.GenericField("Heading");
    public Datalogger.GenericField rP           = new Datalogger.GenericField("Rotation Power");
    public Datalogger.GenericField x            = new Datalogger.GenericField("X pos");
    public Datalogger.GenericField xP           = new Datalogger.GenericField("X power");
    public Datalogger.GenericField y            = new Datalogger.GenericField("Y pos");
    public Datalogger.GenericField yP           = new Datalogger.GenericField("Y power");
    public Datalogger.GenericField targetX      = new Datalogger.GenericField("Target X");
    public Datalogger.GenericField targetY      = new Datalogger.GenericField("Target Y");
    public Datalogger.GenericField targetR      = new Datalogger.GenericField("Target R");
    public Datalogger.GenericField batteryPower = new Datalogger.GenericField("Battery");
    
    Datalogger datalogger = new Datalogger.Builder()
    .setFilename("datalog_02")
    .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)
    .setFields(
        opModeStatus,
        stage,
        yaw,
        x,
        xP,
        y,
        yP,
        targetX,
        targetY,
        targetR,
        batteryPower
    )
    .build();
    
    @Override
    public void runOpMode() throws InterruptedException {
        opModeStatus.set("Init");
        pidController = new PIDController2D(0.05, 0.05, 0.01, 0.05, 0.05, 0.1, 0.1, 0.0, 0.0, 1.75);
        double targetX, targetY;
        claw = hardwareMap.crservo.get("claw");
        battery = hardwareMap.voltageSensor.get("Control Hub");
        slide = new Slide(hardwareMap, this);
        robot = new Drive(hardwareMap, this);

        SparkFunOTOS.Pose2D pos = robot.getPosition();
        
        int stage = 0;
        
        telemetry.addData("xPos", pos.x);
        telemetry.addData("X", 0);
        telemetry.addData("yPos", pos.y);
        telemetry.addData("Y", 0);
        telemetry.addData("stage",stage);
        telemetry.addData("Heading",pos.h);
        telemetry.addData("R", 0);
        
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
            
            opModeStatus.set("AUTON");
            x.set(pos.x);
            //targetX.set((double) outputs[3]);
            y.set(pos.y);
            xP.set(outputX);
            yP.set(outputY);
            yaw.set(pos.h);
            batteryPower.set(battery.getVoltage());
            
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
                    }
                    break;
                case 7:
                    //move to the space between the sub and the samples on the ground
                    pidController.setSpeed(1.0);
                    pidController.setTarget(28, 26, 0);
                    stage++;
                    break;
                case 8:
                    if(pidController.atTarget){
                        stage++;
                        robot.stop();
                    }
                    break;
                case 9:
                    //prepare to push sample(s) into observation zone
                    pidController.setTarget(28, 52, 0);
                    stage++;
                    break;
                case 10:
                    if(pidController.atTarget){
                        stage++;
                        robot.stop();
                    }
                    break;
                case 11:
                    //move over to the sample
                    pidController.setTarget(40, 52, 0);
                    stage++;
                    break;
                case 12:
                    if(pidController.atTarget){
                        stage++;
                        robot.stop();
                    }
                    break;
                case 13:
                    //push first sample
                    pidController.setSpeed(1.0);
                    pidController.setTarget(35,12, 0);
                    break;
                case 14:
                    if(pidController.atTarget){
                        stage++;
                    }
                    break;
                case 15:
                    //get out so human can grab sample
                    pidController.setTarget(35, 28, 0);
                    if(pidController.atTarget){
                        stage++;
                    }
                    break;
                case 16:
                    //move slide to the right level and turn around
                    slide.moveTo(850);
                    robot.rotateTo(180);
                    stage++;
                    break;
                case 17:
                    //go to the wall SLOWLY
                    pidController.setSpeed(2.0);
                    pidController.setTarget(35, 1, 180);
                    stage++;
                case 18:
                    if(pidController.atTarget){
                        stage++;
                    }
                    break;
                //grab & clip x2
            }         
            robot.strafeDrive(outputX, outputY, outputR);
            datalogger.writeLine();
            Thread.sleep(12);
        }
    }    
        

}