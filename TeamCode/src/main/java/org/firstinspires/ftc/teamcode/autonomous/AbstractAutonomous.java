package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.classes.ValueStorage.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.GearheadsMecanumRobotRR;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.PropDetector;

public abstract class AbstractAutonomous extends LinearOpMode {
    public enum Case {
        LEFT, CENTER, RIGHT
    }

    protected GearheadsMecanumRobotRR robot;

    protected TrajectorySequence trajleft, trajCenter, trajRight;
    public static final int minDetected = 10;
    protected PropDetector detector;
    protected Side side;
    protected Case runCase = Case.CENTER;
    protected Case detectCase = Case.CENTER;
    private int detected = 0;

    public abstract void initAutonomous();

    public void initOpMode() {
        robot = new GearheadsMecanumRobotRR(this);
        initAutonomous();
    }

    public void waitOpMode() {

    }


    public void startOpMode() {
        robot.drive.followTrajectorySequenceAsync(trajCenter);
    }

    //@Override
    public void endOpMode() {
        lastSide = side;
        lastPose = robot.drive.getPoseEstimate();
    }
}
