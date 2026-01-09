package org.firstinspires.ftc.teamcode.noncomp.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.configs.LaunchSystem;
import org.firstinspires.ftc.teamcode.configs.LimelightController;
import org.firstinspires.ftc.teamcode.configs.MotorConfig;
import org.firstinspires.ftc.teamcode.configs.ServoConfig;
import org.firstinspires.ftc.teamcode.constants.ServoConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Auto Clean")
public class Auto extends OpMode {
    private LimelightController limelightController;
    private Follower follower;
    private Timer pathTimer;
    private MotorConfig motorConfig;
    private ServoConfig servoConfig;
    private LaunchSystem launchSystem;

    private int pathState = 0;
    private boolean lastB = false;
    private boolean launchStarted = false;
    public static LimelightController.Alliance alliance = LimelightController.Alliance.BLUE;

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose scorePose = new Pose(63, 115, Math.toRadians(150));
    private final Pose fisrtLinePose = new Pose(55, 96, Math.toRadians(180));
    private final Pose pickup1Pose = new Pose(29, 96, Math.toRadians(180));
    private final Pose secondLinePose = new Pose(55, 72, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(24, 72, Math.toRadians(180));
    private final Pose thirdLinePose = new Pose(55, 45, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(24, 45, Math.toRadians(180));

    private PathChain scorePreload, alignRow1, pickupRow1, score1, alignRow2, pickupRow2, score2, alignRow3, pickupRow3, score3;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, new Pose(75, 83), scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading()).build();

        alignRow1 = follower.pathBuilder().addPath(new BezierCurve(scorePose, new Pose(58, 96), fisrtLinePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), fisrtLinePose.getHeading()).build();

        pickupRow1 = follower.pathBuilder().addPath(new BezierLine(fisrtLinePose, pickup1Pose))
                .setLinearHeadingInterpolation(fisrtLinePose.getHeading(), pickup1Pose.getHeading()).build();

        score1 = follower.pathBuilder().addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading()).build();

        alignRow2 = follower.pathBuilder().addPath(new BezierCurve(scorePose, secondLinePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), secondLinePose.getHeading()).build();

        pickupRow2 = follower.pathBuilder().addPath(new BezierLine(secondLinePose, pickup2Pose))
                .setLinearHeadingInterpolation(secondLinePose.getHeading(), pickup2Pose.getHeading()).build();

        score2 = follower.pathBuilder().addPath(new BezierCurve(pickup2Pose, new Pose(60, 60), scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading()).build();

        alignRow3 = follower.pathBuilder().addPath(new BezierLine(scorePose, thirdLinePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), thirdLinePose.getHeading()).build();

        pickupRow3 = follower.pathBuilder().addPath(new BezierLine(thirdLinePose, pickup3Pose))
                .setLinearHeadingInterpolation(thirdLinePose.getHeading(), pickup3Pose.getHeading()).build();

        score3 = follower.pathBuilder().addPath(new BezierCurve(pickup3Pose, new Pose(50, 55), scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading()).build();
    }

    private void performLaunch(int nextState, PathChain nextPath) {
        if (!follower.isBusy()) {
            if (!launchStarted) {
                launchSystem.start(LaunchSystem.HIGH_VELOCITY, 800);
                launchStarted = true;
            }
            if (launchSystem.update()) {
                launchStarted = false;
                if (nextPath != null) follower.followPath(nextPath);
                setPathState(nextState);
            }
        }
    }

    private void performPickup(PathChain pickupPath, int nextState) {
        if (!follower.isBusy()) {
            servoConfig.launchServo.setPosition(ServoConstants.launch_INIT);
            motorConfig.intakeMotor.setPower(1);
            follower.setMaxPower(0.4);
            follower.followPath(pickupPath);
            setPathState(nextState);
        }
    }

    private void returnToScore(PathChain scorePath, int nextState) {
        if (!follower.isBusy()) {
            motorConfig.intakeMotor.setPower(0);
            follower.setMaxPower(1.0);
            follower.followPath(scorePath);
            setPathState(nextState);
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                performLaunch(2, alignRow1);
                break;
            case 2:
                performPickup(pickupRow1, 3);
                break;
            case 3:
                returnToScore(score1, 4);
                break;
            case 4:
                performLaunch(5, alignRow2);
                break;
            case 5:
                performPickup(pickupRow2, 6);
                break;
            case 6:
                returnToScore(score2, 7);
                break;
            case 7:
                performLaunch(8, alignRow3);
                break;
            case 8:
                performPickup(pickupRow3, 9);
                break;
            case 9:
                returnToScore(score3, 10);
                break;
            case 10:
                performLaunch(-1, null);
                break;
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
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void init_loop() {
        if (gamepad1.b && !lastB) {
            alliance = (alliance == LimelightController.Alliance.BLUE) ? LimelightController.Alliance.RED : LimelightController.Alliance.BLUE;
            limelightController.setAlliance(alliance);
        }
        lastB = gamepad1.b;
        telemetry.addData("Alliance", alliance);
        telemetry.update();
    }

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        limelightController.updateTracking();
        handleHood();
        autonomousPathUpdate();
        telemetry.addData("Path State", pathState);
        telemetry.update();
    }

    private void handleHood() {
        double dist = limelightController.getDistance();
        double pos = (dist < 90) ? (0.000235 * dist * dist - 0.03207 * dist + 1.7471) : 0.98;
        servoConfig.hoodServo.setPosition(Range.clip(pos, 0, 1));
    }
}