
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.hardware.GP;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Tele Team Up")
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
        
        double slowModeMod = 1.0;
        boolean slowMode   = false;
        Deadline rateLimit = new Deadline(250, TimeUnit.MILLISECONDS);

        waitForStart();
        
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if(rateLimit.hasExpired() && gamepad1.x){
                if(slowModeMod == 1){
                    slowModeMod = 0.25;
                    slowMode    = true;
                } else if(slowModeMod == 0.25){
                    slowModeMod = 1.0;
                    slowMode    = false;
                }
                rateLimit.reset();
            }

            if((gamepad2.rb && gamepad2.lb) || (gamepad2.up && gamepad2.down)){
                slide.setPower(0);
            } else if((gamepad2.up || gamepad2.rb) && slide.getCurrentPosition() < 10000){
                slide.setPower(1*slowModeMod*2);
            } else if(gamepad2.down  || gamepad2.lb && !(slide.getCurrentPosition()<=50)){
                slide.setPower(-1*slowModeMod*2);
            } else {                                                             
                slide.setPower(0);
            };
            
            if((gamepad1.rb && gamepad1.lb) || (gamepad1.up && gamepad1.down)){
                backArm1.setPower(0);
                backArm2.setPower(0);
            } else if((gamepad1.up || gamepad1.rb) && slide.getCurrentPosition() < 10000){
                backArm1.setPower(1*slowModeMod*2);
                backArm2.setPower(-1*slowModeMod*2);
            } else if(gamepad1.down  || gamepad1.lb && !(slide.getCurrentPosition()<=50)){
                backArm1.setPower(-1*slowModeMod*2);
                backArm2.setPower(1*slowModeMod*2);
            } else {                                                             
                backArm1.setPower(0);
                backArm2.setPower(0);
            };
            
            clawServo.setPower(-gamepad2.lt+gamepad2.rt-0.1);
            assServo.setPower(-gamepad1.lt+gamepad1.rt-0.1);
            
            
            telemetry.addData("pos",slide.getCurrentPosition());
            telemetry.addData("Slowmode",slowMode);
            
            double denominator = Math.max(Math.abs(gamepad1.left_y) + Math.abs(gamepad2.left_x) + Math.abs(gamepad1.right_x), 1);
            double frontLeftPower  = ((gamepad1.left_y + gamepad2.left_x + gamepad1.right_x) / denominator)*slowModeMod;
            double backLeftPower   = ((gamepad1.left_y - gamepad2.left_x + gamepad1.right_x) / denominator)*slowModeMod;
            double frontRightPower = ((gamepad1.left_y - gamepad2.left_x - gamepad1.right_x) / denominator)*slowModeMod;
            double backRightPower  = ((gamepad1.left_y + gamepad2.left_x - gamepad1.right_x) / denominator)*slowModeMod;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            telemetry.update(); 
        }
    }
}

