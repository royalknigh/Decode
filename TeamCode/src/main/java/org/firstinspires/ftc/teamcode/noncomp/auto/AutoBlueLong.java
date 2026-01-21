package org.firstinspires.ftc.teamcode.noncomp.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.configs.LaunchSystem;
import org.firstinspires.ftc.teamcode.configs.LimelightController;
import org.firstinspires.ftc.teamcode.configs.MotorConfig;
import org.firstinspires.ftc.teamcode.configs.ServoConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Auto Blue Long", group = "Autonomous")
public class AutoBlueLong extends OpMode {
    private LimelightController limelightController;
    private Follower follower;
    private Timer pathTimer;
    private MotorConfig motorConfig;
    private ServoConfig servoConfig;
    private LaunchSystem launchSystem;

    private int pathState = 0;

    // --- POSES FROM IMAGE ---
    private final Pose startPose = new Pose(56, 10, Math.toRadians(90));
    private final Pose lineup = new Pose(45, 14, Math.toRadians(90));
    private final Pose pickupPose = new Pose(15, 14, Math.toRadians(180));
    private final Pose scorePose = new Pose(56, 12, Math.toRadians(120));


    private PathChain driveToPickup, driveToScore, pickup;

    public void buildPaths() {
        // Path 1: From Start to Pickup (Intaking while moving)
        driveToPickup = follower.pathBuilder()
                .addPath(new BezierLine(startPose, lineup))
                .addParametricCallback(0, () -> limelightController.toggleTracking())
                .setLinearHeadingInterpolation(startPose.getHeading(), lineup.getHeading())
                .build();
        pickup = follower.pathBuilder()
                .addPath(new BezierLine(lineup, pickupPose))
                .setLinearHeadingInterpolation(lineup.getHeading(), pickupPose.getHeading())
                .addParametricCallback(0,() -> follower.setMaxPower(0.5))
                .addParametricCallback(0, ()-> motorConfig.intakeMotor.setPower(1))
                .build();

        // Path 2: From Pickup back to Scoring Zone
        driveToScore = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose, scorePose))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0,() -> follower.setMaxPower(1))
                .addParametricCallback(0.6, () -> motorConfig.intakeMotor.setPower(0))
                .addParametricCallback(0, () -> limelightController.toggleTracking())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                limelightController.toggleTracking();
                launchSystem.start(LaunchSystem.highVelocity, 800);
                follower.setMaxPower(1);
                setPathState(1);
                break;

            case 1:
                if (launchSystem.update()) {
                    limelightController.toggleTracking();
                    follower.followPath(driveToPickup);
                    launchSystem.fullStop();
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()){
                    follower.followPath(pickup);

                }



        }
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        motorConfig = new MotorConfig(hardwareMap);
        servoConfig = new ServoConfig(hardwareMap);
        launchSystem = new LaunchSystem(motorConfig, servoConfig);
        limelightController = new LimelightController(hardwareMap.get(Limelight3A.class, "limelight"), servoConfig);

        limelightController.setAlliance(LimelightController.Alliance.BLUE);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void loop() {
        follower.update();
        limelightController.updateTracking();
        
        // Always call update to handle the launcher state machine
        launchSystem.update();

        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Flywheel Vel", motorConfig.launchMotor1.getVelocity());
        telemetry.update();
    }

    @Override public void start() { setPathState(0); }
}