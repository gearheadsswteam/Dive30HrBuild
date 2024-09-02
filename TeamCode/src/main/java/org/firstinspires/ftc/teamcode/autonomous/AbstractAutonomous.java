package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.classes.ValueStorage.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.GearheadsMecanumRobotRR;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public abstract class AbstractAutonomous extends LinearOpMode {
    public enum Case {
        LEFT, CENTER, RIGHT
    }

    protected GearheadsMecanumRobotRR robot;

    protected TrajectorySequence trajleft, trajCenter, trajRight;
    public static final int minDetected = 10;

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
//        if (detectCase == detector.getCase()) {
//            detected++;
//        } else {
//            detectCase = detector.getCase();
//            detected = 0;
//        }
//        if (detected >= minDetected) {
//            runCase = detectCase;
//        }

        telemetry.addData("Case Detected", detectCase);
        telemetry.addData("Case to Run", runCase);

    }


    public void startOpMode() {
        //detector.close();
        robot.drive.followTrajectorySequenceAsync(trajCenter);

    }

    //@Override
    public void endOpMode() {
        lastSide = side;
        lastPose = robot.drive.getPoseEstimate();
    }
}
