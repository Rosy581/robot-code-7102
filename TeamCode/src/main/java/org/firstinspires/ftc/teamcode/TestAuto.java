package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@Autonomous(name = "Sensor: GoBilda Pinpoint")
public class TestAuto extends LinearOpMode {
    GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() throws InterruptedException {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        configurePinpoint();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        waitForStart();
        while (opModeIsActive()) {
            pinpoint.update();
            Pose2D pos = pinpoint.getPosition();

            telemetry.addData("X coordinate (IN)", pos.getX(DistanceUnit.INCH));
            telemetry.addData("Y coordinate (IN)", pos.getY(DistanceUnit.INCH));
            telemetry.addData("Heading angle (DEGREES)", pos.getHeading(AngleUnit.DEGREES));
        }
    }
    public void configurePinpoint(){

        pinpoint.setOffsets(0, -6.5, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }
}
