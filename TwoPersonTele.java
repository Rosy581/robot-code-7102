
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.hardware.GP;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Tag Team That Specimen", group = "Robot")
public class TwoPersonTele extends LinearOpMode {

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotor backArm1;
    private DcMotor backArm2;
    private DcMotor slide;
    private DcMotor slideTuah;
    private CRServo clawServo;   
    private Servo assServo;
    private Servo clawTuah;
    private GP lastGp = new GP();
 
    @Override
    
    public void runOpMode() throws InterruptedException {

        frontLeftMotor  = hardwareMap.dcMotor.get("leftFront");
        backLeftMotor   = hardwareMap.dcMotor.get("leftBack");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backRightMotor  = hardwareMap.dcMotor.get("rightBack");
        backArm1 = hardwareMap.dcMotor.get("backArm1");
        backArm2 = hardwareMap.dcMotor.get("backArm2");
        slide = hardwareMap.dcMotor.get("slide");
        slideTuah = hardwareMap.dcMotor.get("slideTuah");
        clawServo = hardwareMap.crservo.get("claw");
        assServo  = hardwareMap.servo.get("assClaw");
        clawTuah  = hardwareMap.servo.get("clawTuah");
        
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backArm2.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideTuah.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideTuah.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backArm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backArm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backArm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        clawServo.setDirection(DcMotorSimple.Direction.REVERSE);
        
        
        double slowModeMod  = 1.0;
        double pos          = 0.5;
        boolean slowMode    = false;
        double targetPos = 0.0;
        Deadline rateLimit  = new Deadline(250, TimeUnit.MILLISECONDS);
        Deadline rateLimit2  = new Deadline(250, TimeUnit.MILLISECONDS);
        
        waitForStart();
        
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y  =  gamepad1.left_stick_y;
            double x  =  -gamepad1.left_stick_x /** 1.1*/;
            double rx =  -gamepad1.right_stick_x;
            if(backArm1.getCurrentPosition() < -1600){
                slideTuah.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if(slideTuah.getCurrentPosition() > -3100) {
                    slideTuah.setPower(gamepad2.left_stick_y);
                } else {
                    slideTuah.setPower(gamepad2.left_stick_y > 0 ? gamepad2.left_stick_y : 0);
                }
                pos = pos > 1 ? 1 : pos;
                pos = pos < 0 ? 0 : pos;
                if(gamepad1.a) {
                    slide.setTargetPosition(850);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION); 
                    slide.setPower(1);
                }
                if(gamepad1.b) {
                    slide.setTargetPosition(2750);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION); 
                    slide.setPower(1);
                }
                if(gamepad2.dpad_up){
                    pos += 0.01;
                } else if (gamepad2.dpad_down){
                    pos -= 0.01;
                }
                if(gamepad2.a){
                    clawTuah.setPosition(1);
                } else {
                    clawTuah.setPosition(0);
                }
                
            } else {
                slideTuah.setTargetPosition(10);
                clawTuah.setPosition(0.5);
                slideTuah.setMode(DcMotor.RunMode.RUN_TO_POSITION); 
                pos = 0.3;
                slideTuah.setPower(1.0);
            }

            
           
            telemetry.addData("pos",pos);
            telemetry.addData("slideTUAH",slideTuah.getCurrentPosition());
            assServo.setPosition(pos);
            
            if(rateLimit.hasExpired() && (gamepad1.x || gamepad2.x)){
                if(slowModeMod == 1){
                    slowModeMod = 0.25;
                    slowMode    = true;
                } else if(slowModeMod == 0.25){
                    slowModeMod = 1.0;
                    slowMode    = false;
                }
                rateLimit.reset();
            }
            
            if(gamepad1.dpad_up && gamepad1.dpad_down){
                slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slide.setPower(0); 
            } else if(gamepad1.dpad_up){
                slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slide.setPower(1*slowModeMod*2);
            } else if(gamepad1.dpad_down){
                slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slide.setPower(-1*slowModeMod*2);
            } else if(!slide.isBusy()){
                slide.setPower(0);
            }

            
            backArm1.setPower(-gamepad2.right_stick_y);
            backArm2.setPower(-gamepad2.right_stick_y);
                
            clawServo.setPower(-gamepad1.left_trigger+gamepad1.right_trigger);

            telemetry.addData("thing",-gamepad1.left_trigger+gamepad1.right_trigger-0.1);
            telemetry.addData("slide pos",slide.getCurrentPosition());
            telemetry.addData("Slowmode",slowMode);
            telemetry.addData("arm",backArm1.getCurrentPosition());
            
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower  = ((y + x + rx) / denominator)*slowModeMod;
            double backLeftPower   = ((y - x + rx) / denominator)*slowModeMod;
            double frontRightPower = ((y - x - rx) / denominator)*slowModeMod;
            double backRightPower  = ((y + x - rx) / denominator)*slowModeMod;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            telemetry.update(); 
            lastGp.update(gamepad1);
        }
    }
}

