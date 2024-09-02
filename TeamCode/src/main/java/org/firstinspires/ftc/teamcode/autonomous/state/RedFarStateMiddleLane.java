package org.firstinspires.ftc.teamcode.autonomous.state;

import static org.firstinspires.ftc.teamcode.autonomous.AbstractAutonomous.Case.CENTER;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.AbstractAutonomous;
import org.firstinspires.ftc.teamcode.classes.ValueStorage;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

//@Photon
@Autonomous(name = "RedFarStateMiddleLane", group = "Red", preselectTeleOp = "TwoDrive")
public class RedFarStateMiddleLane extends AbstractAutonomous {

    public int config = 0;

    public Pose2d startFar = new Pose2d(0, 56, 0);
    public Pose2d dropLeftFar = new Pose2d(39, 58, 5.4);
    public Pose2d intermediatePostDrop = new Pose2d(55, 40, -PI / 2);
    public Pose2d intermediatePostDrop2 = new Pose2d(55,10,-PI / 2);
    public Pose2d board = new Pose2d(30, -25, -PI / 2);
    public Pose2d stackR = new Pose2d(48,79,-PI / 2);

    public Pose2d toDelete = new Pose2d(48,79,-PI / 2);



    private TrajectorySequence initTrajectory(Case caseToRun) {
        side = ValueStorage.Side.RED;
        TrajectorySequenceBuilder builder = robot.drive.trajectorySequenceBuilder(startFar)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(
                        40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(40));
        //Start and deliver purple pixel on left, center or right on tape
        builder.lineToLinearHeading(dropLeftFar)
                .splineTo(intermediatePostDrop.vec(), -PI / 2)
                .splineTo(intermediatePostDrop2.vec(), -PI / 2)
                .waitSeconds(.1)
                .splineTo(board.vec(), -PI/2)
                .waitSeconds(.2)
                .setReversed(true)
                .splineTo(intermediatePostDrop2.vec(), PI / 2)
                .splineTo(intermediatePostDrop.vec(), PI / 2)
                .waitSeconds(.1)
                .splineTo(stackR.vec(),PI / 2)
                .waitSeconds(.2)
                .setReversed(false)
                .splineTo(intermediatePostDrop.vec(), -PI / 2)
                .splineTo(intermediatePostDrop2.vec(), -PI / 2)
                .waitSeconds(.1)
                .splineTo(board.vec(), -PI/2)
                .waitSeconds(.2)
                .setReversed(true)
                .splineTo(intermediatePostDrop2.vec(), PI / 2)
                .splineTo(intermediatePostDrop.vec(), PI / 2)
                .waitSeconds(.1)
                .splineTo(stackR.vec(),PI / 2)
                .waitSeconds(.2)
                .setReversed(false)
                .splineTo(intermediatePostDrop.vec(), -PI / 2)
                .splineTo(intermediatePostDrop2.vec(), -PI / 2)
                .waitSeconds(.1)
                .splineTo(board.vec(), -PI/2)
                .setReversed(true)
                .splineTo(intermediatePostDrop2.vec(), PI / 2)
                .splineTo(intermediatePostDrop.vec(), PI / 2)
                .waitSeconds(.1)
                .splineTo(stackR.vec(),PI / 2)
                .waitSeconds(.2)
                .setReversed(false)
                .splineTo(intermediatePostDrop.vec(), -PI / 2)
                .splineTo(intermediatePostDrop2.vec(), -PI / 2)
                .waitSeconds(.1)
                .splineTo(board.vec(), -PI/2)
                .build();
        return builder.build();
    }

    @Override
    public void initAutonomous() {
        // detector = new PropDetector(this, false, ValueStorage.Side.RED);
        robot.drive.setPoseEstimate(startFar);

        trajCenter = initTrajectory(CENTER);

    }

    @Override
    public void runOpMode() {
        //scheduler = new Scheduler();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        initOpMode();
        while (!isStarted() && !isStopRequested()) {

            waitOpMode();//Team element is detected here last.

            telemetry.update();
        }

        startOpMode();
        while (opModeIsActive()) {

            robot.drive.update();

        }
        endOpMode();
    }
}
