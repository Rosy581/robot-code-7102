import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class ArmTele extends LinearOpMode{
    private DcMotor backArm1;
    private DcMotor backArm2;

    @Override
    public void runOpMode() throws InterruptedException {
        backArm1 = hardwareMap.dcMotor.get("backArm1");
        backArm2 = hardwareMap.dcMotor.get("backArm2");
        waitForStart();
        
        while (opModeIsActive()) {
        double power = gamepad1.left_stick_y-gamepad1.right_stick_y;
        backArm1.setPower(power);
        backArm2.setPower(-power);
    }
        
    }
}
