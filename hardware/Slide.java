package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Slide {
    private DcMotor slide;
    private LinearOpMode opmode;
    public Slide(HardwareMap hardwareMap, LinearOpMode opmode){
        this.slide = hardwareMap.dcMotor.get("slide");
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(1);
        this.opmode = opmode;
    }
        
    public void moveToBottom(){
        slide.setTargetPosition(0);
    }
    public void moveTo(int pos){
        slide.setTargetPosition(pos);
    }
}