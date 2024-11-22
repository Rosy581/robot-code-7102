package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;


public class GP {
    private Gamepad gamepad;
    public GP(Gamepad gamepad){
        this.gamepad = gamepad;
    }
    public boolean x(){
        return gamepad.x;
    }
    public boolean y(){
        return gamepad.y;
    }
    public boolean a(){
        return gamepad.a;
    }
    public boolean b(){
        return gamepad.b;
    }
    public float rt(){
        return gamepad.right_trigger;
    }
    public float lt(){
        return gamepad.left_trigger;
    }
    public boolean rb(){
        return gamepad.right_bumper;
    }
    public boolean lb(){
        return gamepad.left_bumper;
    }
    public boolean up(){
        return gamepad.dpad_up;
    }
    public boolean down(){
        return gamepad.dpad_down;
    }
    public boolean left(){
        return gamepad.dpad_left;
    }
    public boolean right(){
        return gamepad.dpad_right;
    }
    public float left_stick_x(){
        return gamepad.left_stick_x;
    }
    public float left_stick_y(){
        return gamepad.left_stick_y;
    }
    public float right_stick_x(){
        return gamepad.right_stick_x;
    }
    public float left_stick_y(){
        return gamepad.left_stick_y;
    }
    
}
