
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
    private CRServo clawServo;   
    private CRServo assServo;
 
    @Override
    
    public void runOpMode() throws InterruptedException {

        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backRightMotor = hardwareMap.dcMotor.get("rightBack");
        backArm1 = hardwareMap.dcMotor.get("backArm1");
        backArm2 = hardwareMap.dcMotor.get("backArm2");
        slide = hardwareMap.dcMotor.get("slide");
        clawServo = hardwareMap.crservo.get("claw");
        assServo = hardwareMap.crservo.get("assClaw");
        
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backArm2.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backArm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backArm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        clawServo.setDirection(DcMotorSimple.Direction.REVERSE); 
        assServo.setDirection(DcMotorSimple.Direction.REVERSE);
        
        double slowModeMod  = 1.0;
        double sloeModeMod2 = 1.0;
        boolean slowMode    = false;
        boolean slowMode2   = false;
        Deadline rateLimit  = new Deadline(250, TimeUnit.MILLISECONDS);
        Deadline rateLimit2  = new Deadline(250, TimeUnit.MILLISECONDS);

        waitForStart();
        
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y  =  isReversed ? -gamepad1.left_stick_y :  gamepad1.left_stick_y;
            double x  = (isReversed ? gamepad1.left_stick_x  : -gamepad1.left_stick_x) * 1.1;
            double rx =  isReversed ? gamepad1.right_stick_x : -gamepad1.right_stick_x;
            // DIDN'T TEST THE REVERSED STUFF
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
            
            if((gamepad2.right_bumper && gamepad2.left_bumper) || (gamepad2.dpad_up && gamepad2.dpad_down)){
                slide.setPower(0); 
            } else if((gamepad2.dpad_up || gamepad2.right_bumper) && slide.getCurrentPosition() < 10000){
                slide.setPower(1*slowModeMod*2);
            } else if(gamepad2.dpad_down  || gamepad2.left_bumper && !(slide.getCurrentPosition()<=50)){
                slide.setPower(-1*slowModeMod*2);
            } else {                                                             
                slide.setPower(0);
            };
            
            if((gamepad1.right_bumper && gamepad1.left_bumper) || (gamepad1.dpad_up && gamepad1.dpad_down)){
                backArm1.setPower(0);
                backArm2.setPower(0);
            } else if((gamepad1.dpad_up || gamepad1.right_bumper) && slide.getCurrentPosition() < 10000){
                backArm1.setPower(1*slowModeMod*2);
                backArm2.setPower(1*slowModeMod*2);
            } else if(gamepad1.dpad_down  || gamepad1.left_bumper && !(slide.getCurrentPosition()<=50)){
                backArm1.setPower(-1*slowModeMod*2);
                backArm2.setPower(-1*slowModeMod*2);
            } else {                                                             
                backArm1.setPower(0);
                backArm2.setPower(0);
            };
            
            clawServo.setPower(-gamepad2.left_trigger+gamepad2.right_trigger);
            assServo.setPower(-gamepad1.left_trigger+gamepad1.right_trigger);
            
            telemetry.addData("thing",-gamepad1.left_trigger+gamepad1.right_trigger-0.1);
            telemetry.addData("slide pos",slide.getCurrentPosition());
            telemetry.addData("Slowmode (pl. 1)",slowMode);
            telemetry.addData("Slowmode (pl. 2)",slowMode2);

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

