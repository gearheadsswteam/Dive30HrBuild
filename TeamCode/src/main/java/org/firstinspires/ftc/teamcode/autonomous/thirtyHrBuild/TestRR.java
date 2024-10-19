package org.firstinspires.ftc.teamcode.autonomous.thirtyHrBuild;

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

@Autonomous(name = "TestRR", group = "Blue", preselectTeleOp = "TwoDrive")
public class TestRR extends AbstractAutonomous {
    public static Pose2d start = new Pose2d(0, 0, 0);



    private TrajectorySequence initTrajectory() {
        side = ValueStorage.Side.BLUE;

        TrajectorySequenceBuilder builder = robot.drive.trajectorySequenceBuilder(start)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(
                        15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(40));
        //Start and deliver purple pixel on left, center or right on tape
        //What spline path do we need
        builder.forward(24);
              //  .waitSeconds(3);
                //.turn(-90)
                //.back(24);
        return builder.build();
    }

    @Override
    public void initAutonomous() {
        robot.drive.setPoseEstimate(start);
        trajCenter = initTrajectory();
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
