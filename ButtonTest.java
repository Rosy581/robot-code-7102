
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.hardware.GP;

@TeleOp("Button test")
public class ButtonTest extends LinearOpMode {  
    
    public GP lastGP;
    int test = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if(hardware.GP.checkPressed(lastGP.x, gamepad1.x)){
                test++;
            }
            telemetry.addData("val",test);
            lastGP.update(gamepad1);
            telemetry.update();
        }
    }
}

