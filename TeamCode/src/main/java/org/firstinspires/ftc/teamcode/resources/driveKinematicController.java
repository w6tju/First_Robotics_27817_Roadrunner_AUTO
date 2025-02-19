package org.firstinspires.ftc.teamcode.resources;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class driveKinematicController {
    //drive variables
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor rearLeftDrive;
    DcMotor rearRightDrive;
    IMU imu;
    public driveKinematicController() {}

    public void init(HardwareMap hardwareMap) {
        //region Hardware
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        rearLeftDrive = hardwareMap.get(DcMotor.class, "rear_left_drive");
        rearRightDrive = hardwareMap.get(DcMotor.class, "rear_right_drive");
        //endregion

        //region Directions
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotor.Direction.REVERSE);
        //endregion

        //region Zero power
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //endregion
    }


    public void drive(double Travel,double Strafe,double Rotate) {
        //A Drives
        double FL = 0;
        double RR = 0;

        //B Drives
        double FR = 0;
        double RL = 0;

        FL = (Strafe + Travel + Rotate);
        FR = (Strafe - Travel - Rotate);
        RL = (Strafe - Travel + Rotate);
        RR = (Strafe + Travel - Rotate);

        FL = Range.clip(FL,-1,1);
        FR = Range.clip(FR,-1,1);
        RL = Range.clip(RL,-1,1);
        RR = Range.clip(RR,-1,1);

        frontLeftDrive.setPower(FL);
        frontRightDrive.setPower(FR);
        rearLeftDrive.setPower(RL);
        rearRightDrive.setPower(RR);
    }
}

class fieldcentericDriveKinematics extends driveKinematicController {

    @Override
    public void init(HardwareMap hardwareMap) {
        super.init(hardwareMap);
        imu = hardwareMap.get(IMU.class,"imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                        )
                )
        );
    }

    @Override
    public void drive(double travel,double strafe,double Rotate) {
        //Rotational Matrix
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double Travel = travel * Math.cos(-heading) - strafe * Math.sin(-heading);
        double Strafe = travel * Math.cos(-heading) + strafe * Math.sin(-heading);
        super.drive(Travel,Strafe,Rotate);
    }
}
