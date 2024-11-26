
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp
public class TeleDrive extends LinearOpMode {

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotor backArm1;
    private DcMotor backArm2;
    private DcMotor slide;
    private CRServo clawServo;    
 
    @Override

    
    public void runOpMode() throws InterruptedException {

        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backRightMotor = hardwareMap.dcMotor.get("rightBack");
        
        //backArm1 = hardwareMap.dcMotor.get("backArm1");
        //backArm2 = hardwareMap.dcMotor.get("backArm2");
        slide = hardwareMap.dcMotor.get("slide");
        clawServo = hardwareMap.crservo.get("claw");
        
        
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
          
        clawServo.setDirection(DcMotorSimple.Direction.REVERSE); 

        double slowModeMod   = 1.0;
        boolean slowMode   = false;
        Deadline rateLimit = new Deadline(250, TimeUnit.MILLISECONDS);

        waitForStart();
        
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y  =  gamepad1.left_stick_y;
            double x  =  -gamepad1.left_stick_x * 1.1;
            double rx =  -gamepad1.right_stick_x;
            boolean up   = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;
            boolean lB  = gamepad1.left_bumper;
            boolean rB = gamepad1.right_bumper;
            boolean Xbutton = gamepad1.x;
            float leftTrigger  = gamepad1.left_trigger;
            float rightTrigger = gamepad1.right_trigger;
            
            if(rateLimit.hasExpired() && Xbutton){
                if(slowModeMod == 1){
                    slowModeMod = 0.25;
                    slowMode    = true;
                } else if(slowModeMod == 0.25){
                    slowModeMod = 1.0;
                    slowMode    = false;
                }
                rateLimit.reset();
            }

            if((rB && lB) || (up && down)){
                slide.setPower(0);
            } else if((up || rB) && slide.getCurrentPosition() < 10000){
                slide.setPower(1*slowModeMod*2);
            } else if(down  || lB/*&& !(slide.getCurrentPosition()<=50)*/){
                slide.setPower(-1*slowModeMod*2);
            } else {                                                            
                slide.setPower(0);
            };
            
            clawServo.setPower(-leftTrigger+rightTrigger-0.1);

            telemetry.addData("pos",slide.getCurrentPosition());
            telemetry.addData("Slowmode",slowMode);
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

