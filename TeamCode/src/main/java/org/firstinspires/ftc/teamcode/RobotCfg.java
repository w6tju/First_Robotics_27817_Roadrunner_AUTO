package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.Preset;

@Config
public class RobotCfg {
    public static double WRIST_DEBUG = 0;
    public static double WHEEL_SPEED = 1; //Maximum wheel speed (used to slow down the robot, or not)

    //arm configs
    public static int Arm_Pos = 0; //arm setpoint
    public static int Min_Arm = 130; //lowest point arm can travel to
    public static int Max_Arm = 9000; //highest point arm can travel too
    public static int Arm_increment = 10; //Angle (tick) the arm rotates by

    //Viper Slide configs
    public static int Viper_Pos = 0; //Current encoder tick position of the viperslide
    public static int Viper_maxExtend = 3118; //6574 under max upwards, Maximum encoder tick on the viperslide extension
    public static int Viper_minExtend = 0; //Minimum encoder tick on the viperslide extension
    public static int Viper_Increment = 30; //Increment used by viperslide manual extension

    //Pose Storage
    public static Vector2d PoseStorage = new Vector2d(0,0); // used to transfer pose between auto and teleOP

    //Presets
    @Config
    public static class Presets {
        public static Preset Specimen_Intake = new Preset(595,0,0.6);
        public static Preset Enter_Sub = new Preset(4490,0,0.5); // preset for intaking a sample (currently certain death)
        public static Preset Basket_Low = new Preset(0,0,0); // preset for scoring in the low basket
        public static Preset Basket_High = new Preset(1525,0,0.5); // preset for scoring in the high basket
        public static Preset Specemin_Low = new Preset(0,0,0); // preset for scoring a specemin on the low bar
        public static Preset Specemin_High = new Preset(1530,0,0.4); // preset for scoring a specemin on the high bar

    }
}
