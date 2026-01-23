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

@Autonomous(name = "Auto Red Long", group = "Autonomous")
public class AutoRedLong extends OpMode {
    private LimelightController limelightController;
    private Follower follower;
    private Timer pathTimer;
    private MotorConfig motorConfig;
    private ServoConfig servoConfig;
    private LaunchSystem launchSystem;

    private int pathState = 0;

    // --- POSES FROM IMAGE ---
    private final Pose startPose = new Pose(94, 7, Math.toRadians(90));
    private final Pose lineup = new Pose(98, 42, Math.toRadians(0));
    private final Pose pickupPose = new Pose(123, 42, Math.toRadians(0));
    private final Pose leavePose = new Pose(107, 12, Math.toRadians(90));
    private final Pose scorePose = new Pose(89, 12, Math.toRadians(70));


    private PathChain scorePreload, alignRow, pickupRow, score, leave;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .addParametricCallback(0,() -> follower.setMaxPower(1))
                .addParametricCallback(0.1, () -> launchSystem.start(LaunchSystem.highVelocity, 800))
                .addParametricCallback(0, () -> limelightController.toggleTracking())
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        alignRow = follower.pathBuilder()
                .addPath(new BezierLine(startPose, lineup))
                .setLinearHeadingInterpolation(scorePose.getHeading(), lineup.getHeading())
                .build();
        pickupRow = follower.pathBuilder()
                .addPath(new BezierLine(lineup, pickupPose))
                .setLinearHeadingInterpolation(lineup.getHeading(), pickupPose.getHeading())
                .build();
        score = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose, scorePose))
                .addParametricCallback(0,() -> follower.setMaxPower(0.8))
                .addParametricCallback(0.5, () -> launchSystem.start(LaunchSystem.highVelocity, 800))
                .addParametricCallback(0.6, () -> limelightController.toggleTracking())
                .setLinearHeadingInterpolation(pickupPose.getHeading(), scorePose.getHeading())
                .build();
        leave = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, leavePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), leavePose.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                follower.setMaxPower(0.8);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()&&launchSystem.update()) {
                    limelightController.toggleTracking();
                    follower.setMaxPower(0.8);
                    follower.followPath(alignRow);
                    launchSystem.fullStop();
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()){
                    follower.followPath(pickupRow);
                    motorConfig.intakeMotor.setPower(0.9);
                    follower.setMaxPower(0.4);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()){
                    follower.followPath(score);
                    motorConfig.intakeMotor.setPower(0);
                    follower.setMaxPower(0.8);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()&&launchSystem.update()){
                    limelightController.toggleTracking();
                    launchSystem.fullStop();
                    follower.followPath(leave);
                    setPathState(-1);
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

        limelightController.setAlliance(LimelightController.Alliance.RED);
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

    @Override public void init_loop() {
        motorConfig.intakeMotor.setPower(gamepad1.left_trigger);
    }

    @Override public void start() { setPathState(0); }
}