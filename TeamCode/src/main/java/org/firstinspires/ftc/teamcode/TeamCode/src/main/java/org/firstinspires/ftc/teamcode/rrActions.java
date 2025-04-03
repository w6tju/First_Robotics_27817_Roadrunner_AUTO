package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.resources.robotCfg;

public class rrActions {
    org.firstinspires.ftc.teamcode.resources.accessoryControl accessoryControl;


    public rrActions(org.firstinspires.ftc.teamcode.resources.accessoryControl accessory_control) {
        accessoryControl = accessory_control;
    }

    public class intake_Spec implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            accessoryControl.Arm_ToPos(robotCfg.Presets.Specimen_Intake);
            if (accessoryControl.armMotor.getCurrentPosition() < robotCfg.Presets.Specimen_Intake.Arm) {return true;}
            return false;
        }
    }
    public class prep_Hang_Spec implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            accessoryControl.Arm_ToPos(robotCfg.Presets.Specemin_High);
            if (accessoryControl.armMotor.getCurrentPosition() < robotCfg.Presets.Specemin_High.Arm) {return true;}
            return false;
        }
    }

    public class hang_Spec implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robotCfg.Arm_Pos = 1020;
            if (accessoryControl.armMotor.getCurrentPosition() < 1060) {return true;}
            return false;
        }
    }

    public class intakeClass implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            accessoryControl.Set_intake(1);
            return false;
        }
    }

    public class extakeClass implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            accessoryControl.Set_intake(-1);
            return false;
        }
    }
    public class lift_Spec implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robotCfg.Arm_Pos = 1020;
            if (accessoryControl.armMotor.getCurrentPosition() < 1049) {return true;}
            return false;
        }
    }
    public Action intakeSpecimen() {return new intake_Spec();}
    public Action lineupSpecimenHang() {return new prep_Hang_Spec();}
    public Action hangSpecimen() {return new hang_Spec();}
    public Action exhaust() {return new extakeClass();}
    public Action intake() {return new intakeClass();}
    public Action hang() {return new lift_Spec();}

}
