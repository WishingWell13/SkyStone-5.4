package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.BreakIterator;


@Autonomous(name = "Auto_SkyStone_Alt_Blue")
public class Auto_SkyStone_Alt_Blue extends Auto_Abstract {
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


    //Encoder Setup
    static final double PI = Math.PI;
    static final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * PI);


    @Override
    public void runOpMode() {


        generalDefine();

        double grey = getGrayBlue();

        int park = buttonsParkv2();
        int sample = buttonSample();
        double delay = buttonDelay();
        double goToStone = buttonSenseSkystone();
        double stoneSide = buttonSkystonePos();
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
        drive(0.5,22,BACKWARDS);
        //sleep(3000);
        drive(0.4, 75, STRAFE_RIGHT);
        claw(OPEN);
        drive(0.7,3, BACKWARDS);
        drive(0.4,22,STRAFE_LEFT);
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
         * */
        if (sample == YES) {
            claw(OPEN);
            hook(UP);
            drive(0.8, 25, BACKWARDS,GLIDE, true);
            //monoColorDriveSky(0.3,1,FORWARD,RED);
            if(stoneSide == WALL) {
                drive(0.3, 11+24, STRAFE_LEFT, BREAK, true);
                drive(0.4, 7, BACKWARDS, BREAK, true);
            }else{
                drive(0.3, 11, STRAFE_LEFT, BREAK, true);
            }
            int i = 0;
            /*
            Yellow:
            LUM:748
            RED: 264
            Black:
            LUM:540
            RED: 160
             */
            while (colorRev.alpha() >= 600 && opModeIsActive() && (i<2)){ //decreasing threshold if immedietly stopping
                drive(0.2, 11, STRAFE_RIGHT, BREAK, true);
                i++;
                telemetry.addData("Value: ", colorRev.alpha() );
                telemetry.update();
            }
            drive(0.3, 2, STRAFE_RIGHT,BREAK, true);
            drive(0.9, 4, FORWARD,BREAK, true);
            turnDegGyro(LEFT, 187, 0.5);
            claw(PART);
            drive(0.9, 20, FORWARD,BREAK, true);
            claw(CLOSE);
            hook(DOWN);
            if (park==BRIDGE) {
                drive(0.9, 16, BACKWARDS,BREAK, true);
            }else{
                drive(0.9, 35.6, BACKWARDS,BREAK, true);
            }
            turnDegGyro(LEFT, 92, 0.4);
            //drive(0.9, 24, FORWARD, BREAK, false);
            if (stoneSide == WALL) {
                monoColorDrive(0.3, (((grey*0.0) + grey*0.9)), FORWARD, BLUE, 42+20);
            }else {
                monoColorDrive(0.3, (((grey * 0.0) + grey)), FORWARD, BLUE, 42+18);

            }
            //Changing to Color based instead of LUM based
            telemetry.addLine("First Line sensed");
            telemetry.update();
            //sleep(2000);
            drive(0.9, 20, FORWARD, BREAK, true);
            claw(OPEN);
            monoColorDrive(0.3, (((grey *0.0) ) + grey*0.9), BACKWARDS, BLUE, 36);
            //Changing to Color based instead of LUM based
            telemetry.addLine("Second Line sensed");
            telemetry.update();
            //sleep(2000);
            drive(0.1,0.3,FORWARD, BREAK, true);

        } else if (goToStone == YES) {
            claw(PART);
            drive(0.7, 48, FORWARD, GLIDE, false);
            claw(CLOSE);
            if (park == BRIDGE) {
                drive(0.7, 48, BACKWARDS, GLIDE, false);
            } else {
                drive(0.7, 24, FORWARD, GLIDE, false);
            }
            turnDegGyro(LEFT, 90, 0.5);
            monoColorDrive(0.5, (((grey + 0.1) * 1.8) + grey), FORWARD, RED, 42); //Changing to Color based instead of LUM based
            drive(0.8, 15, FORWARD, BREAK, false);
            claw(OPEN);
            monoColorDrive(0.6, (((grey + 0.1) * 1.8) + grey), BACKWARDS, RED, 42); //Sensing for center line
            drive(0.8, 24 + 7 + 4, BACKWARDS, BREAK, false);
            turnDegGyro(RIGHT, 90, 0.5);

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
            turnDegGyro(LEFT, 90, 0.5);
            monoColorDrive(0.5, (((grey + 0.1) * 1.4) + grey), FORWARD, RED, 42); //Changing to Color based instead of LUM based
            drive(0.8, 15, FORWARD, BREAK, false);
            claw(OPEN);
            monoColorDrive(0.6, (((grey + 0.1) * 1.4) + grey), BACKWARDS, RED, 42); //Sensing for center line
            /*drive(0.8, 24+4, BACKWARDS, BREAK, false);
            turnDegGyro(LEFT, 90, 0.5);*/
        }else {
            if (park == BRIDGE) {
                drive(0.5, 30, BACKWARDS, BREAK, false);
            }else{
                drive(0.5, 3, BACKWARDS, BREAK, false);
            }
            monoColorDrive(0.3, (((grey + 0.1) * 1.4) + grey), STRAFE_RIGHT, BLUE, 42); //Changing to Color based instead of LUM based


            // Step 1: Move Forward 30.25 Inches

            // Step 2: Grab The Foundation
            // Step 3: Pull The Foundation Back In To The Depot 29 Inches
            // Step 4: Strafe Left 31 Inches
            //
        }

        //Tiles are 24x24 inches


// todo: write your code here

    }
}