package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Slide {
    private DcMotor slide;
    private LinearOpMode opmode;
    private int target;
    public int tolerance;
    
    public Slide(HardwareMap hardwareMap, LinearOpMode opmode, int tolerance){
        this.slide = hardwareMap.dcMotor.get("slide");
        slide.setTargetPosition(0);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setPower(1);
        this.opmode = opmode;
        this.tolerance = Math.abs(tolerance);
    }
        
    public boolean isBusy(){
        return slide.isBusy();
    }    
    
    public void setSpeed(double speed){
        slide.setPower(speed);
    }

    public  int getPos(){
        return slide.getCurrentPosition();
    }

    public void moveToBottom(){
        slide.setTargetPosition(0);
    }
    
    public boolean atTarget(){
        return target == getPos() || (target - tolerance) < getPos() && getPos() < (target + tolerance);
    }

    public void moveTo(int pos){
        target = pos;
        slide.setTargetPosition(pos);
    }
}