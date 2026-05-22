package org.firstinspires.ftc.teamcode;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.configurables.Config;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Main Auto (Red)", group = "Autonomous")
@Configurable // Panels
public class MainAuto extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Path paths; // Paths defined in the Paths class
    private Robot robot;
    private Timer pathTimer;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Path(follower, Config.autoColor.teamcolor);
        pathTimer = new Timer();
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        robot = new Robot(hardwareMap);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Path{
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6,Path7, Path8,Path9;
        public Path(Follower follower, Robot.TEAMCOLOR teamcolor) {
            if(teamcolor == Robot.TEAMCOLOR.RED) {
                Path1 = follower.pathBuilder().addPath(new BezierLine(new Pose(88.000, 9.000),   new Pose(88.000, 20.000))).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90)).build();
                Path2 = follower.pathBuilder().addPath(new BezierLine(new Pose(88.000, 20.000),  new Pose(88.000, 36.000))).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0)).build();
                Path3 = follower.pathBuilder().addPath(new BezierLine(new Pose(88.000, 36.000),  new Pose(132.000, 36.000))).setTangentHeadingInterpolation().build();
                Path4 = follower.pathBuilder().addPath(new BezierLine(new Pose(132.000, 36.000), new Pose(88.000, 20.000))).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90)).build();
                Path5 = follower.pathBuilder().addPath(new BezierLine(new Pose(88.000, 20.000),  new Pose(88.000, 60.000))).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0)).build();
                Path6 = follower.pathBuilder().addPath(new BezierLine(new Pose(88.000, 60.000),  new Pose(132.000, 60.000))).setTangentHeadingInterpolation().build();
                Path7 = follower.pathBuilder().addPath(new BezierLine(new Pose(132.000, 60.000), new Pose(84.000, 84.000))).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();
                Path8 = follower.pathBuilder().addPath(new BezierLine(new Pose(84.000, 84.000),  new Pose(133.000, 84.000))).setTangentHeadingInterpolation().build();
                Path9 = follower.pathBuilder().addPath(new BezierLine(new Pose(133.000, 84.000), new Pose(84.000, 84.000))).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();
            } else {
                Path1 = follower.pathBuilder().addPath(new BezierLine(new Pose(55.000, 8.000),  new Pose(56.000, 20.000))).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90)).build();
                Path2 = follower.pathBuilder().addPath(new BezierLine(new Pose(56.000, 20.000), new Pose(56.000, 36.000))).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180)).build();
                Path3 = follower.pathBuilder().addPath(new BezierLine(new Pose(56.000, 36.000), new Pose(12.000, 36.000))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();
                Path4 = follower.pathBuilder().addPath(new BezierLine(new Pose(12.000, 36.000), new Pose(56.000, 8.000))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90)).build();
                Path5 = follower.pathBuilder().addPath(new BezierLine(new Pose(56.000, 8.000),  new Pose(56.000, 60.000))).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180)).build();
                Path6 = follower.pathBuilder().addPath(new BezierLine(new Pose(56.000, 60.000), new Pose(12.000, 60.000))).setTangentHeadingInterpolation().build();
                Path7 = follower.pathBuilder().addPath(new BezierLine(new Pose(12.000, 60.000), new Pose(56.000, 8.000))).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90)).build();
                Path8 = follower.pathBuilder().addPath(new BezierLine(new Pose(60.000, 84.000), new Pose(11.000, 84.000))).setTangentHeadingInterpolation().build();
                Path9 = follower.pathBuilder().addPath(new BezierLine(new Pose(11.000, 84.000), new Pose(60.000, 84.000))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();
            }
        }
    }


    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                robot.targetRPM = Config.shooterConstants.targetRPMfar;
                follower.followPath(paths.Path1);
                next();
            case 1:
                if(!follower.isBusy()) {
                    robot.aimAtPos(Robot.BlueCorner);
                    robot.shoot();
                    robot.toggleIntake();//on
                    next();
                }
            case 2:
                if(pathTimer.getElapsedTimeSeconds()>=3) {
                    robot.toggleIntake();//off
                    robot.shoot();
                    follower.followPath(paths.Path2);
                    next();
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    robot.toggleIntake();//on
                    follower.followPath(paths.Path3);
                    next();
                }
            case 4:
                if(!follower.isBusy()){
                    follower.followPath(paths.Path4);
                    robot.toggleIntake();//off
                    next();
                }
                break;
            case 5:
                robot.aimAtPos(Robot.BlueCorner);
                robot.shoot();
                robot.toggleIntake();//on
                next();
                break;
            default:
                return -2;
        }
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return -1;
    }

    private void next() {
        pathState += 1;
        pathTimer.resetTimer();
    }
}