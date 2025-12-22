package org.firstinspires.ftc.teamcode.noncomp.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.configs.MotorConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Auto")
public class Auto extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState = 0;
//    private MotorConfig motorConfig;

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose scorePose = new Pose(40, 120, Math.toRadians(150));
    private final Pose fisrtLinePose = new Pose(45, 95, Math.toRadians(180));
    private final Pose pickup1Pose = new Pose(29, 95, Math.toRadians(180));
    private final Pose secondLinePose = new Pose(46, 68, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(24, 68, Math.toRadians(180));
    private final Pose thirdLinePose = new Pose(46, 45, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(16, 45, Math.toRadians(180));

    public PathChain scorePreload, alignRow1, pickupRow1, score1, alignRow2, pickupRow2, score2, alignRow3, pickupRow3, score3;

    public void buildPaths() {

        scorePreload = follower
                .pathBuilder()
                .addPath(new BezierCurve(startPose, new Pose(75.000, 83.000), scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        alignRow1 = follower
                .pathBuilder()
                .addPath(new BezierCurve(scorePose,new Pose(57.000, 96.000), fisrtLinePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), fisrtLinePose.getHeading())
                .build();

        pickupRow1 = follower
                .pathBuilder()
                .addPath(new BezierLine(fisrtLinePose, pickup1Pose))
                .setLinearHeadingInterpolation(fisrtLinePose.getHeading(), pickup1Pose.getHeading())
                .build();

        score1 = follower
                .pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        alignRow2 = follower
                .pathBuilder()
                .addPath(new BezierCurve(scorePose, secondLinePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), secondLinePose.getHeading())
                .build();

        pickupRow2 = follower
                .pathBuilder()
                .addPath(new BezierLine(secondLinePose, pickup2Pose))
                .setLinearHeadingInterpolation(secondLinePose.getHeading(), pickup2Pose.getHeading())
                .build();

        score2 = follower
                .pathBuilder()
                .addPath(new BezierCurve(pickup2Pose,new Pose(60.000, 60.000),  scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        alignRow3 = follower
                .pathBuilder()
                .addPath(new BezierLine(scorePose, thirdLinePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), thirdLinePose.getHeading())
                .build();

        pickupRow3 = follower
                .pathBuilder()
                .addPath(new BezierLine(thirdLinePose, pickup3Pose))
                .setLinearHeadingInterpolation(thirdLinePose.getHeading(), pickup3Pose.getHeading())
                .build();

        score3 = follower
                .pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
//                    MotorConfig.launchMotor.setPower(1);        //start launchMotor
//                    if(pathTimer.getElapsedTimeSeconds()>=5){   //wait x seconds
//                        MotorConfig.launchMotor.setPower(1);    //stop launchMotor
                        follower.followPath(alignRow1);
                        setPathState(2);
//                    }
                }
                break;
            case 2:
                if (!follower.isBusy()) {
//                    MotorConfig.inatkeMotor.setPower(1);    //start intakeMotor
                    follower.followPath(pickupRow1);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
//                    MotorConfig.inatkeMotor.setPower(0);    //stop intakeMotor
                    follower.followPath(score1);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
//                    MotorConfig.launchMotor.setPower(1);        //start launchMotor
//                    if(pathTimer.getElapsedTimeSeconds()>=5){   //wait x seconds
//                        MotorConfig.launchMotor.setPower(1);    //stop launchMotor
                        follower.followPath(alignRow2);
                        setPathState(5);
//                    }
                }
                break;
            case 5:
                if (!follower.isBusy()) {
//                    MotorConfig.inatkeMotor.setPower(1);    //start intakeMotor
                    follower.followPath(pickupRow2);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
//                    MotorConfig.inatkeMotor.setPower(0);    //stop intakeMotor
                    follower.followPath(score2);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
//                    MotorConfig.launchMotor.setPower(1);        //start launchMotor
//                    if(pathTimer.getElapsedTimeSeconds()>=5){   //wait x seconds
//                        MotorConfig.launchMotor.setPower(1);    //stop launchMotor
                        follower.followPath(alignRow3);
                        setPathState(8);
//                    }
                }
                break;
            case 8:
                if (!follower.isBusy()) {
//                    MotorConfig.inatkeMotor.setPower(1);    //start  intakeMotor
                    follower.followPath(pickupRow3);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
//                    MotorConfig.inatkeMotor.setPower(0);    //stop intakeMotor
                    follower.followPath(score3);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
//                    MotorConfig.launchMotor.setPower(1);        //start launchMotor
//                    if(pathTimer.getElapsedTimeSeconds()>=5){   //wait x seconds
//                        MotorConfig.launchMotor.setPower(1);    //stop launchMotor
                        setPathState(-1);
                    }
//                }
                break;


        }
    }

    public void setPathState(int state){
        pathState=state;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}

}
