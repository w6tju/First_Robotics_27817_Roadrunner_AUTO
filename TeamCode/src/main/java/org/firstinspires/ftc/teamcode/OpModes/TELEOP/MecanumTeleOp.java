/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.OpModes.TELEOP;

import static org.firstinspires.ftc.teamcode.resources.robotCfg.*;
import static org.firstinspires.ftc.teamcode.resources.accessoryControl.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.resources.accessoryControl;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.resources.chassisKinematics;
import org.firstinspires.ftc.teamcode.resources.chassisKinematics.controlRelativity;
import org.firstinspires.ftc.teamcode.resources.driveKinematicController;


@TeleOp()
@Config
public class MecanumTeleOp extends LinearOpMode {
    //Hybrid Telemetry
    Telemetry Telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    boolean loader;
    //Declare OpMode members.
    ElapsedTime runtime = new ElapsedTime();
    boolean last30 = false;
    boolean last10 = false;
    boolean last10Buzz = false;
    double lastBuzzTime;
    driveKinematicController controller;
    controlRelativity controlMode = controlRelativity.Field;

    @Override
    public void runOpMode() {
        ThreeDeadWheelLocalizer localizer = new ThreeDeadWheelLocalizer(hardwareMap, MecanumDrive.PARAMS.inPerTick,new Pose2d(20, -62, Math.toRadians(90)));
        accessoryControl accessoryController = new accessoryControl(hardwareMap,false);
        chassisKinematics chassisKinematics = new chassisKinematics();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //waits for start button on driver hub\\
        waitForStart();
        accessoryController.atStart();
        runtime.reset();
        //Main loop (runs as long as you are in op mode)\\
        while (opModeIsActive()) {
            WHEEL_SPEED = Range.clip(WHEEL_SPEED,-1,1);
            //region Inputs
            double Drive,Drift,Turn;
            Turn = -gamepad1.right_stick_x * Math.abs(WHEEL_SPEED);// steering input
            if (controlMode == controlRelativity.Field) {
                Drift = gamepad1.left_stick_x * Math.abs(WHEEL_SPEED); // forward input
                Drive = gamepad1.left_stick_y * Math.abs(WHEEL_SPEED);// strafe input
                controller.fieldCentericDrive(Drive,Drift,Turn);
            }
            else {
                Drift = -gamepad1.left_stick_x * WHEEL_SPEED; // forward input
                Drive = gamepad1.left_stick_y * WHEEL_SPEED; // strafe input
                controller.drive(Drive,Drift,Turn); //drive output
            }
            //endregion

            accessoryController.RunAccessory(gamepad2,gamepad1);
            localizer.update();

            //region Telemetry
            //Telemetry (shows up on driver hub and FTCDashboard)\\
            Telemetry.addData("Status", "Run Time: " + runtime.toString());
            Telemetry.addData("Position: ",localizer.getPose());
            Telemetry.addData("Arm: ",Arm_Pos);
            Telemetry.addData("ViperSlide: ",Viper_Pos);
            if (!Intake_Active) {
                Telemetry.addData("Intake: ", "Ejecting");
            }
            else {
                Telemetry.addData("Intake: ","Intaking");
            }
            Telemetry.addData("Arm_Mode: ", accessoryControl.Arm_Mode);
            Telemetry.update();
            //endregion
        }
    }
}