
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

        double slowModeMod   = 1.0;
        boolean slowMode   = false;
        Deadline rateLimit = new Deadline(250, TimeUnit.MILLISECONDS);

        waitForStart();
        
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            GP gp1 = new GP(gamepad1);
            GP gp2 = new GP(gamepad2);
            if(rateLimit.hasExpired() && X1){
                if(slowModeMod == 1){
                    slowModeMod = 0.25;
                    slowMode    = true;
                } else if(slowModeMod == 0.25){
                    slowModeMod = 1.0;
                    slowMode    = false;
                }
                rateLimit.reset();
            }

            if((gp2.rb && gp2.lb) || (gp2.up && gp2.down)){
                slide.setPower(0);
            } else if((gp2.up || gp2.rB) && slide.getCurrentPosition() < 10000){
                slide.setPower(slowModeMod*2);
            } else if(gp2.down2  || lB2 && !(slide.getCurrentPosition()<=50)){
                slide.setPower(-slowModeMod*2);
            } else {                                                             
                slide.setPower(0);
            };
            
            clawServo.setPower(-leftTrigger1+rightTrigger1-0.1);

            telemetry.addData("pos",slide.getCurrentPosition());
            telemetry.addData("Slowmode",slowMode);
            
            double denominator = Math.max(Math.abs(y1) + Math.abs(x1) + Math.abs(rx), 1);
            double frontLeftPower  = ((y1 + x1 + rx) / denominator)*slowModeMod;
            double backLeftPower   = ((y1 - x1 + rx) / denominator)*slowModeMod;
            double frontRightPower = ((y1 - x1 - rx) / denominator)*slowModeMod;
            double backRightPower  = ((y1 + x1 - rx) / denominator)*slowModeMod;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            telemetry.update(); 
        }
    }
}

