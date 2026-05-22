package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.configurables.Config.autoColor.teamcolor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.configurables.Config;

@Autonomous
public class SimpleAuto extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        robot.configureMotorsZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.configOdo();
        double x = Config.autoColor.teamcolor == Robot.TEAMCOLOR.BLUE ? 54.5 : 89.5;
        robot.setPosition(new Pose2D(DistanceUnit.INCH, x, 8.5, AngleUnit.DEGREES, 0));
        robot.update(teamcolor);
        int state = 0;
        waitForStart();
        while (opModeIsActive()) {
            switch (state){
                case 0:
                    robot.aimAtPos(teamcolor);
                    robot.targetRPM = Config.shooterConstants.targetRPMfar + 250;
                    state++;
                    break;
                case 1:
                    robot.shoot();
                    robot.toggleIntake();
                    state++;
                    break;
                case 2:
                    Thread.sleep(3000);
                    state++;
                    break;
                /*
                case 3:
                    robot.mecanumDrive(0,1,0);
                    Thread.sleep(1000);
                    state++;
                    break;
                case 4:
                    robot.stop();
                    break;*/
            }
            robot.update(teamcolor);
            telemetry.addData("loc",robot.getPos());
            telemetry.addData("turret rotation", robot.getTurretRotation());
            telemetry.update();
        }
    }
}