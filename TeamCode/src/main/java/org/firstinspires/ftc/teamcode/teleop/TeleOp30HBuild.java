package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.classes.ValueStorage.side;
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
import org.firstinspires.ftc.teamcode.classes.ValueStorage;
import org.firstinspires.ftc.teamcode.robot.GearheadsMecanumRobotRR;

@TeleOp(name = "TeleOpRedBlue30HR")
public class TeleOp30HBuild extends LinearOpMode {

    Robot robot = new Robot();
    GearheadsMecanumRobotRR gearheadsMecanumRobotRR;



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

}
