package org.firstinspires.ftc.teamcode.actionParts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private Servo grip;
    private Servo turn;

    private int gripOpen = 0;
    private int gripClosed = 0 ;

    private int turn90Degree = 0 ;

    public Claw(LinearOpMode opMode) {
        grip = opMode.hardwareMap.get(Servo.class, "grip"); //TODO: change to config
        turn = opMode.hardwareMap.get(Servo.class, "turn");
    }

    public void openClaw() {
        grip.setPosition(gripOpen);
    }

    public void closeClaw() {
        grip.setPosition(gripClosed);
    }

    public void turn() {
        double curPostion = turn.getPosition();
        turn.setPosition(curPostion + turn90Degree);
    }
}
