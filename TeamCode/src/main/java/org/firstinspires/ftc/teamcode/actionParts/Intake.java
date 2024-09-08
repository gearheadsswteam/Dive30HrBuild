package org.firstinspires.ftc.teamcode.actionParts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private DcMotorEx activeIntakeMotor;
    private DcMotorEx intakeArmMotor;
    private Servo clawServo;

    private int intakeGrab = 0;
    private int intakeDeliver = 0 ;

    private int clawOpen = 0;
    private int clawClosed = 0 ;

    public Intake(LinearOpMode opMode) {
        activeIntakeMotor = opMode.hardwareMap.get(DcMotorEx.class, "motor1"); //TODO: change to config
        intakeArmMotor = opMode.hardwareMap.get(DcMotorEx.class, "motor2");
        clawServo = opMode.hardwareMap.get(Servo.class, "servo1");

        intakeArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeArmMotor.setTargetPosition(0);
        intakeArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void openClaw() {
        clawServo.setPosition(clawOpen);
    }

    public void closeClaw() {
        clawServo.setPosition(clawClosed);
    }

    public void startIntake() {
        activeIntakeMotor.setPower(0.5);

    }

    public void stopIntake() {
        activeIntakeMotor.setPower(0.0);

    }

    public void reverseIntake() {
        activeIntakeMotor.setPower(-0.5);

    }

    public void goToDeliveryPosition(){
        intakeArmMotor.setTargetPosition(intakeDeliver);
    }

    public void goToIntakePosition(){
        intakeArmMotor.setTargetPosition(intakeGrab);
    }
}
