package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.PI;

@Autonomous(name = "Auto_SkyStone_Alt_Red")
public class Auto_SkyStone_Alt_Red extends Auto_Abstract {
    DcMotor lf, rf, lb, rb, ls;
    public Gamepad g1, g2;
    Servo clawL, clawR, hook;
    private ElapsedTime runtime = new ElapsedTime();
    //Directions
    static final int FORWARD = 0;
    static final int BACKWARDS = 1;
    static final int STRAFE_RIGHT = 2;
    static final int STRAFE_LEFT = 3;
    static final int FREEFORM = 4;
    static final int UP = 0;
    static final int DOWN = 1;

    static final int WALL = 0;
    static final int BRIDGE = 1;

    static final int YES = 0;
    static final int NO = 1;

    public static boolean LEFT = false;
    public static boolean RIGHT = true;

    Orientation lastAngles = new Orientation();
    double globalAngle = .30;


    //Encoder Setup
    // static final double PI = Math.PI;
    static final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * PI);



    @Override
    public void runOpMode() {


        generalDefine();
        double grey = getGrayRedRev();
        int park = buttonsParkv2();
        int sample = buttonSample();
        int goToStone = buttonSenseSkystone();
        int blockSide = buttonSkystonePos();
        double delay = buttonDelay();
        telemetry.addLine("Done");
        telemetry.update();


        //----------------------------------------------------------------------------------------------------------------//
        //Foundation Only
        //Move 120 Inches

        waitForStart();
        telemetry.addData("Waiting", "...");
        telemetry.update();
        sleep((long) delay);
        /*sleep(5000);
        claw(PART);
        drive(0.5,33, FORWARD);
        claw(CLOSE);
        //sleep(3000);
        drive(0.5,25,BACKWARDS);
        //sleep(3000);
        drive(0.4,87,STRAFE_LEFT);
        claw(OPEN);
        drive(0.4,22,STRAFE_RIGHT);
        */

        //Skystone & Foundation Theory
        /*Create 2 autonomouses: 1 for left skystone and one for right skystone
         * Need Color Sensonr
         * Left skystone: Start from left and scan until hit block that isn't yellow
         * Right skystone: Start from right and scan until hit block that isn't yellow
         *
         * Take one skystone to foundation, let other team take other one
         * One team does foundation, one does not
         * Park
         *
         * //Yellow Block Red Reading: 191
        //Yellow Block Green reading: 300
        //No Block Red Reading: 118
        //Skystone Red Reading: 112
         * */
        if (sample == YES) {
            claw(OPEN);
            hook(DOWN);
            drive(0.6, 25.6, BACKWARDS, GLIDE, false);
            //encoderTurn180(0.5);
            //monoColorDriveSky(0.3,1,FORWARD,RED);
            if (blockSide == WALL){
                turnDegGyro(RIGHT, 90,0.6);
                drive(0.3, 36, FORWARD,BREAK,false);
            }
            int i = 0;
            while ((colorRev.alpha() >= 600) && opModeIsActive() && (i < 2)) { //decreasing threshold if immedietly stopping
                i++;
                drive(0.2, 11, STRAFE_LEFT, BREAK, false);
            } //Changed to Rev color sensor 1/29/2020
            drive(0.3, 8, STRAFE_RIGHT, BREAK, false);

            drive(0.9, 6, FORWARD, BREAK, false);
            //drive(0.9, 5, FORWARD);
            turnDegGyro(LEFT, 174, 0.6);
            claw(PART);
            drive(0.7, 25, FORWARD, BREAK, false);
            claw(CLOSE);
            if (park == BRIDGE) {
                drive(0.9, 16, BACKWARDS, BREAK, false);
            } else {
                drive(0.9, 43, BACKWARDS, BREAK, false);
            }
            turnDegGyro(RIGHT, 93, 0.7);
            drive(0.9, 20, FORWARD, BREAK, false);
            //drive(0.8, 30, FORWARD, BREAK, false);
            monoColorLineRev(0.3, grey, FORWARD, RED, 48-8*i); //Changing to Color based instead of LUM based
            //decrease threshold if going on forever

            drive(0.8, 15, FORWARD, BREAK, false);
            claw(OPEN);
            monoColorLineRev(0.3, grey, BACKWARDS, RED, 15); //Sensing for center line
            //drive(0.8, 30, FORWARD, BREAK, false);
            //Changing to Color based instead of LUM based
            drive(0.1, 0.01, FORWARD, BREAK, false);

        } else if (goToStone == YES) {
            claw(PART);
            drive(0.7, 48, FORWARD, GLIDE, false);
            claw(CLOSE);
            if (park == BRIDGE) {
                drive(0.7, 48, BACKWARDS, GLIDE, false);
            } else {
                drive(0.7, 24, FORWARD, GLIDE, false);
            }
            turnDegGyro(RIGHT, 90, 0.5);
            monoColorDrive(0.5, (((grey + 0.1) * 1.4) + grey), FORWARD, RED, 42); //Changing to Color based instead of LUM based
            drive(0.8, 15, FORWARD, BREAK, false);
            claw(OPEN);
            monoColorDrive(0.6, (((grey + 0.1) * 1.4) + grey), BACKWARDS, RED, 42); //Sensing for center line
            drive(0.8, 24 + 7 + 4, BACKWARDS, BREAK, false);
            turnDegGyro(LEFT, 90, 0.5);

            //Block Two
            telemetry.addLine("Block Two");
            telemetry.update();
            drive(0.7, 48, FORWARD, GLIDE, false);
            claw(CLOSE);
            if (park == BRIDGE) {
                drive(0.7, 48, BACKWARDS, GLIDE, false);
            } else {
                drive(0.7, 24, FORWARD, GLIDE, false);
            }
            turnDegGyro(RIGHT, 90, 0.5);
            monoColorDrive(0.5, (((grey + 0.1) * 1.4) + grey), FORWARD, RED, 42); //Changing to Color based instead of LUM based
            drive(0.8, 15, FORWARD, BREAK, false);
            claw(OPEN);
            monoColorDrive(0.6, (((grey + 0.1) * 1.4) + grey), BACKWARDS, RED, 42); //Sensing for center line
            /*drive(0.8, 24+4, BACKWARDS, BREAK, false);
            turnDegGyro(LEFT, 90, 0.5);*/

        } else {
            if (park == BRIDGE) {
                drive(0.5, 29, BACKWARDS, BREAK, false);
            } else {
                drive(0.5, 5, BACKWARDS, BREAK, false);
            }
            monoColorDrive(0.3, (((grey + 0.1) * 1.4) + grey), STRAFE_LEFT, RED, 42); //Changing to Color based instead of LUM based
            sleep(500);
        }
    }
//updated 9:41 12/14/2019





    // Step 1: Move Forward 30.25 Inches

    // Step 2: Grab The Foundation
    // Step 3: Pull The Foundation Back In To The Depot 29 Inches
    // Step 4: Strafe Left 31 Inches
    //
}

//Tiles are 24x24 inches




// todo: write your code here
