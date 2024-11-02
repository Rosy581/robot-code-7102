import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class TeleDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");
        DcMotor slide = hardwareMap.dcMotor.get("slide");
        CRServo clawServo = hardwareMap.crservo.get("claw");
        
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        clawServo.setDirection(DcMotorSimple.Direction.REVERSE);
        
        waitForStart();
        
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y  =   gamepad1.left_stick_y;
            double x  =  -gamepad1.left_stick_x * 1.1;
            double rx =  -gamepad1.right_stick_x;
            boolean up   = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;
            boolean xPressed = gamepad1.X;
            double slowModeMod = 1.0;
            float leftTrigger  = gamepad1.left_trigger;
            float rightTrigger = gamepad1.right_trigger;
            
            if(slowModeMod == 1 && xPressed){
                slowModeMod = 1/4; 
            } else if(slowModeMod == 1/4 && xPressed){
                slowModeMod = 1.0;
            }

            if(down && up){    
                slide.setPower(0);
            } else if(up){
                slide.setPower(-1);
            } else if(down&& !(slide.getCurrentPosition()>=-50)){
                slide.setPower(1);
            } else {
                slide.setPower(0);
            };
            
            clawServo.setPower(-leftTrigger+rightTrigger);

            telemetry.addData("pos",slide.getCurrentPosition());
            telemetry.addData("lTrigger", leftTrigger);
            telemetry.addData("rTrigger", rightTrigger);
            telemetry.addData("X",x);
            telemetry.addData("y",y);
            telemetry.addData("rx",rx);
            
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = ((y + x + rx) / denominator)*slowModeMod;
            double backLeftPower = (( y - x + rx) / denominator)*slowModeMod;
            double frontRightPower = ((y - x - rx) / denominator)*slowModeMod;
            double backRightPower = ((y + x - rx) / denominator)*slowModeMod;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            telemetry.update(); 
        }
    }
}
