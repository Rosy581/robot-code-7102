package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name = "Sensor: GoBilda Pinpoint")
public class TestAuto extends LinearOpMode {
    GoBildaPinpointDriver pinpoint;
    Robot robot = new Robot(this, hardwareMap);
    @Override
    public void runOpMode() throws InterruptedException {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        waitForStart();
        while (opModeIsActive()) {
            pinpoint.update();
            Pose2D pos = robot.odo.getPosition();

            telemetry.addData("X coordinate (IN)", pos.getX(DistanceUnit.INCH));
            telemetry.addData("Y coordinate (IN)", pos.getY(DistanceUnit.INCH));
            telemetry.addData("Heading angle (DEGREES)", pos.getHeading(AngleUnit.DEGREES));
        }
    }

}
