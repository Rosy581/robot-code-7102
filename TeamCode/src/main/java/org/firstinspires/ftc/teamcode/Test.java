package org.firstinspires.ftc.teamcode;

import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

@TeleOp(name="Test", group="Tele")

public class Test extends LinearOpMode {


    double kp = 0.0;
    double k  = 0.0;
    double output = 0.0;
    double tollerance = 100;
    private ElapsedTime runtime = new ElapsedTime();
    double power = 0.5;
    double lastTime = 0;
    double curTime = 0;
    double prevPos = 0;
    @Override
    public void runOpMode() {

        DcMotor shooter  = hardwareMap.get(DcMotor.class, "shooter");

        Deadline rateLimit  = new Deadline(250, TimeUnit.MILLISECONDS);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            if(rateLimit.hasExpired() && (gamepad1.dpad_up)){
                power = power + 0.05;
                rateLimit.reset();
            }
            if(rateLimit.hasExpired() && (gamepad1.dpad_down)){
                power = power - 0.05;
                rateLimit.reset();
            }
            curTime = getRuntime();
            telemetry.addData("RPM",(Math.abs((shooter.getCurrentPosition()) - prevPos)/28)*600);
            telemetry.addData("Power",power);
            telemetry.addData("Output",output);
            shooter.setPower(output);
            lastTime = getRuntime();
            prevPos = shooter.getCurrentPosition();
            telemetry.update();
            sleep(100);
        }
    }
}
