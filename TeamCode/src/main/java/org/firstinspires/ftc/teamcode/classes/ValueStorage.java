package org.firstinspires.ftc.teamcode.classes;
// import com.acmerobotics.roadrunner.geometry.Pose2d;
import static com.qualcomm.robotcore.util.Range.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class ValueStorage {
    public static int signalMinCount = 10;
    public static int side = sides.RED;
    public static class sides {
        public static final int RED = 1;
        public static final int BLUE = -1;
    }
    public static Pose2d lastPose = new Pose2d(0, 0, 0);

    public static double liftKp = 0.02;
    public static double liftKi = 0;
    public static double liftKd = 0;

    public static double liftKf (double input) {
        return 0.0003 * input;
    }
}