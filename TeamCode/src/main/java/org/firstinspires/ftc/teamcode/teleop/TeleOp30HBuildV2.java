package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.classes.Robot;
import org.firstinspires.ftc.teamcode.robot.GearheadsMecanumRobotRR;

@TeleOp(name = "TeleOpRedBlue30HRV2")
public class TeleOp30HBuildV2 extends LinearOpMode {

    Robot robot = new Robot();
    GearheadsMecanumRobotRR gearheadsMecanumRobotRR;

    private int armState = 0;
    ElapsedTime armStateTime = new ElapsedTime();


    //double initialHeading = ValueStorage.lastPose.getHeading() - side * PI / 2;
    double initialHeading = 0;
    double robotHeading;
    double moveAngle;
    double moveMagnitude;
    double turn;
    double time;

    ElapsedTime clock = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        gearheadsMecanumRobotRR = new GearheadsMecanumRobotRR(this);
        gearheadsMecanumRobotRR.initTeleop(hardwareMap);

        while (!opModeIsActive() && !isStopRequested()) {
            time = clock.seconds();
            //robot.update(time);
        }


        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.ps) {
                initialHeading -= robotHeading;
            }
            time = clock.seconds();


            moveRobot();
            liftStatemachine();


        }
    }


    private void moveRobot() {
        robotHeading = robot.getHeading() + initialHeading;
        moveAngle = atan2(-gamepad1.left_stick_x, -gamepad1.left_stick_y) - robotHeading;
        moveMagnitude = abs(pow(gamepad1.left_stick_x, 3)) + abs(pow(gamepad1.left_stick_y, 3));
        if (moveMagnitude < 0.01) {
            moveMagnitude = 0;
        }
        turn = pow(gamepad1.right_stick_x, 3);
        if (abs(turn) < 0.01) {
            turn = 0;
        }
        float speedDamper = gamepad1.right_trigger;

        if (speedDamper < 0.3) {
            robot.setDrivePowers(moveMagnitude * Range.clip(sin(PI / 4 - moveAngle) / abs(cos(PI / 4 - moveAngle)), -1, 1) + turn,
                    moveMagnitude * Range.clip(sin(PI / 4 + moveAngle) / abs(cos(PI / 4 + moveAngle)), -1, 1) - turn,
                    moveMagnitude * Range.clip(sin(PI / 4 + moveAngle) / abs(cos(PI / 4 + moveAngle)), -1, 1) + turn,
                    moveMagnitude * Range.clip(sin(PI / 4 - moveAngle) / abs(cos(PI / 4 - moveAngle)), -1, 1) - turn);
        } else {
            double speedDamperFactor = 0.3;
            robot.setDrivePowers(speedDamperFactor * (moveMagnitude * Range.clip(sin(PI / 4 - moveAngle) / abs(cos(PI / 4 - moveAngle)), -1, 1) + turn),
                    speedDamperFactor * (moveMagnitude * Range.clip(sin(PI / 4 + moveAngle) / abs(cos(PI / 4 + moveAngle)), -1, 1) - turn),
                    speedDamperFactor * (moveMagnitude * Range.clip(sin(PI / 4 + moveAngle) / abs(cos(PI / 4 + moveAngle)), -1, 1) + turn),
                    speedDamperFactor * (moveMagnitude * Range.clip(sin(PI / 4 - moveAngle) / abs(cos(PI / 4 - moveAngle)), -1, 1) - turn));

        }
    }


    private void liftStatemachine() {
        telemetry.addLine("Arm  state HERE " + armState);
        telemetry.update();

        // while (true) {
        if (gamepad1.x) {
            // if statement to replace switch
            if (armState == 0) {
                if (armStateTime.milliseconds() > 2000) {
                    gearheadsMecanumRobotRR.elevator.goInitPosition();
                    gearheadsMecanumRobotRR.claw.openClaw();

                    armState = 1;
                    armStateTime.reset();
                }

            } else if (armState == 1) {
                if (armStateTime.milliseconds() > 2000) {

                    gearheadsMecanumRobotRR.claw.closeClaw();

                    armState = 2;
                    armStateTime.reset();
                }

            } else if (armState == 2) {
                if (armStateTime.milliseconds() > 1000) {
                    gearheadsMecanumRobotRR.elevator.goHighBasket();

                    armState = 3;
                    armStateTime.reset();
                }


            } else if (armState == 3) {
                if (armStateTime.milliseconds() > 2000) {
                    gearheadsMecanumRobotRR.claw.openClaw();

                    armState = 0;
                    armStateTime.reset();
                }
            }
        }
    }
}
