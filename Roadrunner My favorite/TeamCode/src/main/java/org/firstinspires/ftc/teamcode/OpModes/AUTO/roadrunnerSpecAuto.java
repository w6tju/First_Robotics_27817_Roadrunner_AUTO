package org.firstinspires.ftc.teamcode.OpModes.AUTO;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AccessoryControl;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotCfg.Presets;
import static org.firstinspires.ftc.teamcode.RobotCfg.*;
import org.firstinspires.ftc.teamcode.rrActions;

import java.util.ServiceConfigurationError;
import java.util.concurrent.CompletableFuture;

@Autonomous()
public class roadrunnerSpecAuto extends LinearOpMode {
    //Hybrid Telemetry
    Telemetry Telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    AccessoryControl accessoryControl;
    rrActions rrActions;
    Servo wrist;
    Vector2d prepPickupPose = new Vector2d(46,-55);
    Vector2d pickupPose = new Vector2d(46,-60);
    double dropHeading = Math.toRadians(90);
    double pickupHeading = Math.toRadians(-90);

    @Override
    public void runOpMode() {
        accessoryControl = new AccessoryControl(hardwareMap,true);
        rrActions = new rrActions(accessoryControl);
        MecanumDrive drive = new MecanumDrive(hardwareMap,new Pose2d(20, -61.9, Math.toRadians(90)));

        Action preload = drive.actionBuilder(new Pose2d(20, -62, Math.toRadians(90)))
                .strafeTo(new Vector2d(0,-34.4), new TranslationalVelConstraint(45)) //Preload  Specimen
                .stopAndAdd(rrActions.hangSpecimen())
                .waitSeconds(0.1)
                .stopAndAdd(rrActions.exhaust())
                .strafeTo(new Vector2d(0,-56))
                .build();
        Action beginLineup = drive.actionBuilder(new Pose2d(0,-30,Math.toRadians(90)))
                .strafeTo(new Vector2d(47,-51.34)) //begin lineup
                .strafeTo(new Vector2d(47.38, -11))
                .build();
        Action sample1 = drive.actionBuilder(new Pose2d(47,-10,Math.toRadians(90)))
                .strafeTo(new Vector2d(52.1,-10.32)) //Sample 1
                .strafeTo(new Vector2d(51.33,-58.5))
                .strafeTo(new Vector2d(52.,-10.3))
                .build();
        Action sample2 = drive.actionBuilder(new Pose2d(46,-10,Math.toRadians(90)))
                .strafeTo(new Vector2d(61.45,-10.35), new TranslationalVelConstraint(30)) //Sample 2
                .strafeTo(new Vector2d(61.45,-58.5))
                .strafeToLinearHeading(new Vector2d(50.45,-5.35),Math.toRadians(180))
                .build();
        Action sample3 = drive.actionBuilder(new Pose2d(50,-10,Math.toRadians(180)))
                .strafeTo(new Vector2d(67.28,-10.52)) //Sample 3
                .strafeTo(new Vector2d(70.40,-65.83))
                .strafeTo(new Vector2d(67.28,-20.5))
                .build();
        Action specimen1 = drive.actionBuilder(new Pose2d(63,-20,Math.toRadians(180)))
                .stopAndAdd(rrActions.intakeSpecimen())
                .strafeToLinearHeading(new Vector2d(59.58266202697216,-38.55245483577456),pickupHeading)
                .strafeToLinearHeading(new Vector2d(59.58266202697216,-59),pickupHeading, new TranslationalVelConstraint(30))
                .waitSeconds(.5)
                .stopAndAdd(rrActions.intakeSpecimen())
                .strafeToLinearHeading(new Vector2d(59.58266202697216,-64),pickupHeading, new TranslationalVelConstraint(30))
                .stopAndAdd(rrActions.intake())
                .waitSeconds(.2)
                .stopAndAdd(rrActions.lineupSpecimenHang())
                .stopAndAdd(rrActions.intake())
                .waitSeconds(.5)
                .setTangent(Math.toRadians(135))
                .strafeToLinearHeading(new Vector2d(7.58266202697216,-55),pickupHeading)
                .strafeToLinearHeading(new Vector2d(-2.571291340548382,-45.260942531416916),dropHeading)
                .strafeToLinearHeading(new Vector2d(-2.571291340548382,-26),dropHeading)
                .stopAndAdd(rrActions.hangSpecimen())
                .waitSeconds(0.9)
                .stopAndAdd(rrActions.exhaust())
                .stopAndAdd(rrActions.hang())
                .build();
        Action specimen2 = drive.actionBuilder(new Pose2d(36,-20,dropHeading))
                .strafeToLinearHeading(new Vector2d(-2.571291340548382,-37),pickupHeading)
                .stopAndAdd(rrActions.intakeSpecimen())
                .strafeToLinearHeading(new Vector2d(-2.571291340548382,-40),pickupHeading)
                .strafeToLinearHeading(new Vector2d(85.58266202697216,-69),pickupHeading)
                .waitSeconds(.5)
                .stopAndAdd(rrActions.intakeSpecimen())
                .strafeToLinearHeading(new Vector2d(85.58266202697216,-72),pickupHeading, new TranslationalVelConstraint(30))
                .stopAndAdd(rrActions.intake())
                .stopAndAdd(rrActions.lineupSpecimenHang())
                .stopAndAdd(rrActions.intake())
                .waitSeconds(.5)
                .strafeToLinearHeading(new Vector2d(7.58266202697216,-55),pickupHeading)
                .strafeToLinearHeading(new Vector2d(-3.6,-45.260942531416916),dropHeading)
                .strafeToLinearHeading(new Vector2d(-3.6,-26),dropHeading)
                .stopAndAdd(rrActions.hangSpecimen())
                .waitSeconds(0.9)
                .stopAndAdd(rrActions.exhaust())
                .build();
        Action specimen3 = drive.actionBuilder(new Pose2d(2,-40,dropHeading))
                .stopAndAdd(rrActions.intakeSpecimen())
                .strafeToLinearHeading(new Vector2d(50.58266202697216,-38.55245483577456),pickupHeading)
                .strafeToLinearHeading(new Vector2d(54.58266202697216,-55),pickupHeading)
                .stopAndAdd(rrActions.intake())
                .stopAndAdd(rrActions.lineupSpecimenHang())
                .strafeToLinearHeading(new Vector2d(-5.271291340548382,-43.260942531416916),dropHeading)
                .stopAndAdd(rrActions.hangSpecimen())
                .waitSeconds(0.2)
                .stopAndAdd(rrActions.exhaust())
                .build();
        Action specimen4 = drive.actionBuilder(new Pose2d(-2,-40,dropHeading))
                .stopAndAdd(rrActions.intakeSpecimen())
                .strafeToLinearHeading(new Vector2d(50.58266202697216,-38.55245483577456),pickupHeading)
                .strafeToLinearHeading(new Vector2d(52.58266202697216,-55),pickupHeading)
                .stopAndAdd(rrActions.intake())
                .stopAndAdd(rrActions.lineupSpecimenHang())
                .strafeToLinearHeading(new Vector2d(-10.271291340548382,-43.260942531416916),dropHeading)
                .stopAndAdd(rrActions.hangSpecimen())
                .waitSeconds(0.2)
                .stopAndAdd(rrActions.exhaust())
                .build();
        Action park = drive.actionBuilder(new Pose2d(-6,-40,dropHeading))
                .strafeTo(new Vector2d(60,-60))
                .build();
        wrist = hardwareMap.get(Servo.class,"wrist");
        wrist.setPosition(1);
        waitForStart();
        CompletableFuture.runAsync(() -> {
            while (opModeIsActive()) {
                accessoryControl.Run_Motors();
            }
        });

        Actions.runBlocking(
                new SequentialAction(
                        rrActions.lineupSpecimenHang(),
                        preload,
                        rrActions.hangSpecimen(),
                        beginLineup,
                        sample1,
                        sample2,
                        //sample3,
                        rrActions.intakeSpecimen(),
                        specimen1,
                        specimen2
                        // specimen3
                        //specimen4
                )
        );
    }

    void pickupSpec(MecanumDrive drive, Pose2d startPose) {
        accessoryControl.Arm_ToPos(Presets.Specimen_Intake);
        Actions.runBlocking(
            drive.actionBuilder(startPose)
                    .strafeToLinearHeading(prepPickupPose,pickupHeading)
                    .strafeToLinearHeading(pickupPose,pickupHeading)
                    .strafeToLinearHeading(prepPickupPose,pickupHeading)
                    .build()
        );
        accessoryControl.Arm_ToPos(Presets.Specemin_High);
    }

    void leaveSub(MecanumDrive drive, Pose2d startPose) {
        Arm_Pos = 900;
        drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(startPose.position.x,startPose.position.y-10))
                .build();
    }
}