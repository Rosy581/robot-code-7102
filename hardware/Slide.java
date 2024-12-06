package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Slide {
    private DcMotor slide;
    private LinearOpMode opmode;
    public Slide(DcMotor slide, LinearOpMode opmode){
        this.slide = slide;
        this.opmode = opmode;
    }
        
    public void moveToBottom(){
        slide.setTargetPosition(0);
        slide.setPower(1);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(opmode.opModeIsActive() && slide.isBusy()){

        }
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setPower(0);
    }
    
    public void moveToTop(){
        slide.setTargetPosition(10000);
        slide.setPower(1);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(opmode.opModeIsActive() && slide.isBusy()){

        }
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setPower(0);
    }
    public void moveTo(int pos){
        slide.setTargetPosition(pos);
        int power = pos>slide.getCurrentPosition()?1:-1;
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(power);
                while(opmode.opModeIsActive() && slide.isBusy()){

        }
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setPower(0);
    }
}