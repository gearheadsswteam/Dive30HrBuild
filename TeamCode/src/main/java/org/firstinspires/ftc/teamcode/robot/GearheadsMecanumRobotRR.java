package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.actionParts.Claw;
import org.firstinspires.ftc.teamcode.actionParts.Elevator;
import org.firstinspires.ftc.teamcode.actionParts.Light;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.sparkFunOTOS.SparkFunOTOSDrive;


/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 * <p>
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class GearheadsMecanumRobotRR {

    private LinearOpMode curOpMode = null;   //current opmode

    public SampleMecanumDrive drive;

    /* local OpMode members. */
    public HardwareMap hwMap = null;

    public Claw claw;

    public Elevator elevator;

    public Light light;

    /* Constructor */
    public GearheadsMecanumRobotRR(LinearOpMode opMode) {
        this.curOpMode = opMode;
        hwMap = opMode.hardwareMap;
        //drive = new SampleMecanumDrive(hwMap);
        //TODO - use SparkFun OTOS Drive
        drive = new SparkFunOTOSDrive(opMode,hwMap);


    }


    /* Initialize standard Hardware interfaces */
    public void initTeleop(HardwareMap ahwMap) {
        init(ahwMap);
    }

    /* Initialize standard Hardware interfaces */
    public void initAutonomous(HardwareMap ahwMap) {
        init(ahwMap);
    }


    private void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
//        claw = new Claw(curOpMode);
//        elevator = new Elevator();
        light = new Light(curOpMode);

    }
}


