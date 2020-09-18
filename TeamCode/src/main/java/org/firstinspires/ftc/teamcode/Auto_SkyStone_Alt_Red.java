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
import static java.lang.Math.nextDown;

@Autonomous(name = "Auto_SkyStone_Alt_Red_Comp")
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
        int noSensor2Stone = buttonSenseSkystone();
        int twoStone = button2StoneSense();
        int blockSide = buttonSkystonePos();
        double delay = buttonDelay();
        telemetry.addLine("Done (Comp)");
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
            hook(UP);
            capServo(DOWN);
            drive(0.6, 26.5, BACKWARDS, GLIDE, true);
            //encoderTurn180(0.5);
            //monoColorDriveSky(0.3,1,FORWARD,RED);
            if (blockSide == WALL){
                /*turnDegGyro(RIGHT, 90,0.6);
                drive(0.3, 8, FORWARD,BREAK,false);
                turnDegGyro(LEFT, 90,0.6);
                */
                drive(0.4, 10, STRAFE_RIGHT, BREAK,false);
            }
            int i = 0;
            if (blockSide == BRIDGE) {
                while ((colorRev.alpha() >= 270) && opModeIsActive() && (i < 2)) { //decreasing threshold if immedietly stopping
                    i++;
                    drive(0.2, 11, STRAFE_LEFT, BREAK, false);
                } //Changed to Rev color sensor 1/29/2020
            }else{
                while ((colorRev.alpha() >= 270) && opModeIsActive() && (i < 2)) { //decreasing threshold if immedietly stopping
                    i++;
                    drive(0.2, 8, STRAFE_RIGHT, BREAK, false);
                    telemetry.addData("Lum: ", colorRev.alpha());
                    telemetry.addData("Block Place: ", i);
                    telemetry.update();
                } //Changed to Rev color sensor 1/29/2020
            }
             //Debug
            telemetry.addData("Lum: ", colorRev.alpha());
            telemetry.addData("Block Place: ", i);
            telemetry.update();
            sleep(2000);
            ///------------------ Position to Pick Up Block
            if(blockSide == BRIDGE) {
                drive(0.3, 6.5, STRAFE_RIGHT, BREAK, false);
            }else{
                if (i!=2) {
                    drive(0.3, 3, STRAFE_RIGHT, BREAK, false);
                }else {
                    drive(0.3, 3, STRAFE_LEFT, BREAK, false);
                }
            }
            //-------------------- Turn Around
            drive(0.7, 6, FORWARD, BREAK, false);
            //drive(0.9, 5, FORWARD);
            if (blockSide == BRIDGE) {
                turnDegGyro(LEFT, 180, 0.6);
            }else{
                if(i!=2) {
                    turnDegGyro(LEFT, 183, 0.6);
                }else{
                    turnDegGyro(LEFT, 195, 0.6);
                }
            }
            //--------------------- Pick up block
            claw(PART);
            drive(0.7, 27, FORWARD, BREAK, true);
            claw(CLOSE);
            //---------------------- Back UP
            if (park == BRIDGE) {
                drive(0.8, 22, BACKWARDS, BREAK, true);
            } else {
                drive(0.8, 43, BACKWARDS, BREAK, true);
            }
            //------------------ Turn To Bridge
            if (blockSide ==BRIDGE) {
                turnDegGyro(RIGHT, 91, 0.7);
            }else{
                if(i!=2) {
                    turnDegGyro(RIGHT, 100, 0.6);
                }else{
                    turnDegGyro(RIGHT, 110, 0.6);
                }
            }

            hook(DOWN);
            //Line Up With Bridge
            if (park == BRIDGE && blockSide == WALL){
                drive(0.7, 7.5, STRAFE_LEFT, BREAK, false);
            }else if (park == BRIDGE && blockSide == BRIDGE){
                drive(0.7, 1, STRAFE_LEFT, BREAK, false);
            }else if (park == WALL && blockSide == BRIDGE){
                drive(0.7, 7.5, STRAFE_RIGHT, BREAK, false);
            }

            drive(0.6, 20, FORWARD, BREAK, false);
            //drive(0.8, 30, FORWARD, BREAK, false);
            if (blockSide == BRIDGE) {
                monoColorLineRev(0.3, grey * 1.1, FORWARD, RED, 50 - (8 * i)); //Changing to Color based instead of LUM based
            }else {
                monoColorLineRev(0.3, grey * 1.1, FORWARD, RED, 50 + (9 * i)); //Changing to Color based instead of LUM based
            }
            //decrease threshold if going on forever

            drive(0.8, 15, FORWARD, BREAK, true);
            claw(OPEN);
            monoColorLineRev(0.3, grey*1.05, BACKWARDS, RED, 24); //Sensing for center line
            //drive(0.8, 30, FORWARD, BREAK, false);
            //Changing to Color based instead of LUM based
            drive(0.1, 0.01, FORWARD, BREAK, false);

        } else if (twoStone == YES){
            claw(OPEN);
            hook(UP);
            capServo(DOWN);
            drive(0.7, 26.5, BACKWARDS, GLIDE, true);
            //encoderTurn180(0.5);
            //monoColorDriveSky(0.3,1,FORWARD,RED);
            if (blockSide == WALL){
                /*turnDegGyro(RIGHT, 90,0.6);
                drive(0.3, 8, FORWARD,BREAK,false);
                turnDegGyro(LEFT, 90,0.6);
                */
                drive(0.5, 10, STRAFE_RIGHT, BREAK,false);
            }
            int i = 0;
            if (blockSide == BRIDGE) {
                while ((colorRev.alpha() >= 150) && opModeIsActive() && (i < 2)) { //decreasing threshold if immedietly stopping
                    i++;
                    drive(0.4, 11, STRAFE_LEFT, BREAK, false);
                } //Changed to Rev color sensor 1/29/2020
            }else{
                while ((colorRev.alpha() >= 140) && opModeIsActive() && (i < 2)) { //decreasing threshold if immedietly stopping
                    i++;
                    drive(0.4, 8, STRAFE_RIGHT, BREAK, false);
                    telemetry.addData("Lum: ", colorRev.alpha());
                    telemetry.addData("Block Place: ", i);
                    telemetry.update();
                } //Changed to Rev color sensor 1/29/2020
            }
            //Debug
            telemetry.addData("Lum: ", colorRev.alpha());
            telemetry.addData("Block Place: ", i);
            telemetry.update();
            sleep(2000);
            ///------------------ Position to Pick Up Block
            if(blockSide == BRIDGE) {
                drive(0.4, 6.5, STRAFE_RIGHT, BREAK, false);
            }else{
                if (i!=2) {
                    drive(0.4, 3, STRAFE_RIGHT, BREAK, false);
                }else {
                    drive(0.4, 3, STRAFE_LEFT, BREAK, false);
                }
            }
            //-------------------- Turn Around
            drive(0.7, 6, FORWARD, BREAK, false);
            //drive(0.9, 5, FORWARD);
            if (blockSide == BRIDGE) {
                turnDegGyro(LEFT, 180, 0.6);
            }else{
                if(i!=2) {
                    turnDegGyro(LEFT, 183, 0.6);
                }else{
                    turnDegGyro(LEFT, 195, 0.6);
                }
            }
            //--------------------- Pick up block
            claw(PART);
            drive(0.8, 27, FORWARD, BREAK, true);
            claw(CLOSE);
            //---------------------- Back UP
            if (park == BRIDGE) {
                drive(0.8, 22, BACKWARDS, BREAK, true);
            } else {
                drive(0.8, 43, BACKWARDS, BREAK, true);
            }
            //------------------ Turn To Bridge
            if (blockSide ==BRIDGE) {
                turnDegGyro(RIGHT, 91, 0.7);
            }else{
                if(i!=2) {
                    turnDegGyro(RIGHT, 100, 0.6);
                }else{
                    turnDegGyro(RIGHT, 110, 0.6);
                }
            }

            hook(DOWN);
            //Line Up With Bridge
            if (park == BRIDGE && blockSide == WALL){
                drive(0.7, 7.5, STRAFE_LEFT, BREAK, false);
            }else if (park == BRIDGE && blockSide == BRIDGE){
                drive(0.7, 1, STRAFE_LEFT, BREAK, false);
            }else if (park == WALL && blockSide == BRIDGE){
                drive(0.7, 7.5, STRAFE_RIGHT, BREAK, false);
            }

            drive(0.6, 20, FORWARD, BREAK, false);
            //drive(0.8, 30, FORWARD, BREAK, false);
            if (blockSide == BRIDGE) {
                monoColorLineRev(0.6, grey * 1.1, FORWARD, RED, 50 - (8 * i)); //Changing to Color based instead of LUM based
            }else {
                monoColorLineRev(0.6, grey * 1.1, FORWARD, RED, 50 + (9 * i)); //Changing to Color based instead of LUM based
            }
            //decrease threshold if going on forever

            drive(0.8, 15, FORWARD, BREAK, true);
            claw(OPEN);
            monoColorLineRev(0.6, grey*1.05, BACKWARDS, RED, 24); //Sensing for center line
            //drive(0.8, 30, FORWARD, BREAK, false);
            //Changing to Color based instead of LUM based
            drive(0.1, 0.01, FORWARD, BREAK, false);



            //-----------------------------------------Stone Two------------------------------------------------
            turnDegGyro(RIGHT,180, 0.6);
            if (blockSide == WALL) {
                drive(0.6, 25+9*i, FORWARD, BREAK, false);
            }else {
                drive(0.6, 48+9*i, FORWARD, BREAK, false);
            }
            turnDegGyro(RIGHT, 90, 0.4);
            //--------------------- Pick up block
            claw(PART);
            drive(0.7, 27, FORWARD, BREAK, true);
            claw(CLOSE);
            //---------------------- Back UP
            if (park == BRIDGE) {
                drive(0.8, 22, BACKWARDS, BREAK, true);
            } else {
                drive(0.8, 43, BACKWARDS, BREAK, true);
            }
            //------------------ Turn To Bridge
            if (blockSide ==BRIDGE) {
                turnDegGyro(RIGHT, 91, 0.7);
            }else{
                if(i!=2) {
                    turnDegGyro(RIGHT, 100, 0.6);
                }else{
                    turnDegGyro(RIGHT, 110, 0.6);
                }
            }

            hook(DOWN);
            //Line Up With Bridge
            if (park == BRIDGE && blockSide == WALL){
                drive(0.7, 7.5, STRAFE_LEFT, BREAK, false);
            }else if (park == BRIDGE && blockSide == BRIDGE){
                drive(0.7, 1, STRAFE_LEFT, BREAK, false);
            }else if (park == WALL && blockSide == BRIDGE){
                drive(0.7, 7.5, STRAFE_RIGHT, BREAK, false);
            }

            drive(0.6, 20, FORWARD, BREAK, false);
            //drive(0.8, 30, FORWARD, BREAK, false);
            if (blockSide == BRIDGE) {
                monoColorLineRev(0.3, grey * 1.1, FORWARD, RED, 50 - (8 * i)); //Changing to Color based instead of LUM based
            }else {
                monoColorLineRev(0.3, grey * 1.1, FORWARD, RED, 50 + (9 * i)); //Changing to Color based instead of LUM based
            }
            //decrease threshold if going on forever

            drive(0.8, 15, FORWARD, BREAK, true);
            claw(OPEN);
            monoColorLineRev(0.6, grey*1.05, BACKWARDS, RED, 24); //Sensing for center line
            //drive(0.8, 30, FORWARD, BREAK, false);
            //Changing to Color based instead of LUM based
            drive(0.1, 0.01, FORWARD, BREAK, false);


        }else if (noSensor2Stone == YES) {
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
            monoColorLineRev(0.3, grey , STRAFE_LEFT, RED, 42); //Changing to Color based instead of LUM based
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
