package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
import org.firstinspires.ftc.teamcode.resources.PIDController2D;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

@Autonomous(name = "TEST", group = "Robot")

public class Test extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private CRServo claw;
    private Slide slide;
    private Drive robot;
    private PIDController2D pidController;
    SparkFunOTOS myOtos;
    Deadline rateLimit = new Deadline(250, TimeUnit.MILLISECONDS);
    
    @Override
    public void runOpMode() throws InterruptedException {
        pidController = new PIDController2D(0.1, 0.0, 0.0, 0.1, 0.0, 0.0);
        double targetX, targetY;
        myOtos = hardwareMap.get(SparkFunOTOS.class, "otos");
        claw = hardwareMap.crservo.get("claw");
        //slide = new Slide(hardwareMap, this);
        robot = new Drive(hardwareMap, this);
        
        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        
        myOtos.setLinearScalar(1.03125);
        myOtos.setAngularScalar(1.0);
        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.DEGREES);
        
        myOtos.calibrateImu();
        myOtos.resetTracking();
        
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);
        
        telemetry.addData("xPos", pos.x);
        telemetry.addData("yPos", pos.y);
        
        telemetry.update();
        
        waitForStart();
        while (opModeIsActive()) {
            pidController.setSetpoints(0, 0);
            pos = myOtos.getPosition();
            double currentX  = pos.x;
            double currentY  = pos.y;
            double[] outputs = pidController.calculate(currentX, currentY);
            double outputX   = outputs[0];
            double outputY   = outputs[1];
            //telemetry.addData("Entered",outputs[3]);
            //telemetry.addData("Entered",outputs[2]);
            telemetry.addData("X",outputX);
            telemetry.addData("xPos", pos.x);
            telemetry.addData("yPos", pos.y);
            telemetry.addData("Y",outputY);
            telemetry.update();
            Thread.sleep(25);
            robot.strafeDrive(outputX, outputY, 0.0);
        } 
        
        sleep(25000);
    }   
}
 