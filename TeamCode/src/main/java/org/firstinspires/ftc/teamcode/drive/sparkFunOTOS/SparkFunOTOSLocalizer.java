package org.firstinspires.ftc.teamcode.drive.sparkFunOTOS;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

import java.util.List;

public class SparkFunOTOSLocalizer extends StandardTrackingWheelLocalizer {
    SparkFunOTOS sparkFunOTOS;
    LinearOpMode curOpMode;
    public SparkFunOTOSLocalizer(LinearOpMode opMode, SparkFunOTOS sparkFunOTOS, HardwareMap hardwareMap, List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels) {
        super(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels);
        curOpMode = opMode;
        this.sparkFunOTOS = sparkFunOTOS;
    }

    public SparkFunOTOSLocalizer(SparkFunOTOS sparkFunOTOS, HardwareMap hardwareMap) {
        super(hardwareMap,null,null);
        this.sparkFunOTOS = sparkFunOTOS;
    }


    @Override
    public void update() {
        //Use SparkFunOTOS to get poseEstimate and poseVelocity
        SparkFunOTOS.Pose2D position = sparkFunOTOS.getPosition();
        super.setPoseEstimate(new Pose2d(position.x, position.y, Math.toRadians(position.h)));
        SparkFunOTOS.Pose2D velocity = sparkFunOTOS.getVelocity();
        super.setPoseVelocity(new Pose2d(velocity.x, velocity.y, Math.toRadians(velocity.h)));


        // Log the position to the telemetry
        curOpMode.telemetry.addData("X coordinate", position.x);
        curOpMode.telemetry.addData("Y coordinate", position.y);
        curOpMode.telemetry.addData("Heading angle", position.h);

        // Update the telemetry on the driver station
        curOpMode.telemetry.update();

    }
}
