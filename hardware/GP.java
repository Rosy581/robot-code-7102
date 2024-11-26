package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;


public class GP {
    private Gamepad gamepad;
    public GP(Gamepad gamepad){
        this.gamepad = gamepad;
    }
    public boolean x = gamepad.x;
    public boolean y = gamepad.y;
    public boolean a = gamepad.a;
    public boolean b = gamepad.b;
    public float rt = gamepad.right_trigger;
    public float lt = gamepad.left_trigger;
    public boolean rb = gamepad.right_bumper;
    public boolean lb = gamepad.left_bumper;
    public boolean up = gamepad.dpad_up;
    public boolean down = gamepad.dpad_down;
    public boolean left = gamepad.dpad_left;
    public boolean right = gamepad.dpad_right;
    public float left_x = gamepad.left_stick_x;
    public float left_y = gamepad.left_stick_y;
    public float right_x = gamepad.right_stick_x;
    public float right_y = gamepad.right_stick_y;
}