package org.firstinspires.ftc.teamcode.OpModes.AUTO;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.AccessoryControl;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous
public class PedroAUTO extends LinearOpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    Pose start = new Pose(0,0,0);
    Pose preload_Specemin = new Pose(54.86,12.65,0.04);
    Pose before_Preset3 = new Pose(41.64,-44.24,0.04);
    Pose before_First = new Pose(116.36,49.67,0.04);
    Pose load_First = new Pose(102.09,-77.13,0.04);
    Pose observation_First = new Pose(12.27,-50.21,0.04);
    Pose before_Second = new Pose(102.09,-91.92,0.04);
    Pose observation_Second = new Pose(12.27,-67.07,0.04);
    Pose pickup_Specemin = new Pose(11.86,86.30,0.08);
    PathChain score_preload, collect_First, collect_Second, score_First, score_Second;

    public void build_Paths() {
        score_preload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(start),new Point(preload_Specemin)))
                .setLinearHeadingInterpolation(start.getHeading(),preload_Specemin.getHeading())
                .build();
        collect_First = follower.pathBuilder()
                .addPath(new BezierLine(new Point(preload_Specemin),new Point(before_Preset3)))
                .addPath(new BezierLine(new Point(before_Preset3),new Point(before_First)))
                .addPath(new BezierLine(new Point(before_First),new Point(load_First)))
                .addPath(new BezierLine(new Point(load_First),new Point(observation_First)))
                .setLinearHeadingInterpolation(preload_Specemin.getHeading(),before_Preset3.getHeading())
                .build();
        collect_Second = follower.pathBuilder()
                .addPath(new BezierLine(new Point(observation_First),new Point(load_First)))
                .addPath(new BezierLine(new Point(load_First),new Point(before_Second)))
                .addPath(new BezierLine(new Point(before_Second),new Point(observation_Second)))
                .setLinearHeadingInterpolation(observation_First.getHeading(),observation_Second.getHeading())
                .build();
        score_First = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup_Specemin),new Point(preload_Specemin)))
                .setLinearHeadingInterpolation(pickup_Specemin.getHeading(), preload_Specemin.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(score_preload);
                setPathState(1);
                break;
            case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(collect_First,true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(collect_Second,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(score_First,true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(score_Second,true);
                    setPathState(5);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void runOpMode() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        AccessoryControl accessoryControl = new AccessoryControl(hardwareMap,false);
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(0,0,0));
        build_Paths();

        waitForStart();
        accessoryControl.atStart();
        while (opModeIsActive()) {
            autonomousPathUpdate();
            accessoryControl.Run_Motors();
            follower.update();
        }
    }
}
