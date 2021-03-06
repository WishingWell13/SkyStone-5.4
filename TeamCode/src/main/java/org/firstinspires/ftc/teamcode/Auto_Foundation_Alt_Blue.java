package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Auto_Foundation_Alt_Blue_Comp")
public class Auto_Foundation_Alt_Blue extends Auto_Abstract{
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


    //Encoder Setup
    static final double PI = Math.PI;
    static final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * PI);



    @Override
    public void runOpMode(){

        telemetry.addData("Check -1", "...");
        telemetry.update();
        generalDefine();
        telemetry.addData("Check 0", "...");
        telemetry.update();
        double grey = getGrayBlueRev();
        telemetry.addData("Check 1", "...");
        telemetry.update();
        double delay = buttonDelay();
        telemetry.addData("Check 2", "...");
        telemetry.update();
        int park = buttonsParkv2();
        telemetry.addData("Check 3", "...");
        telemetry.update();
        int foundation = buttonFoundation();
        telemetry.addData("Done", "...");
        telemetry.update();

        //----------------------------------------------------------------------------------------------------------------//
        //Foundation Only
        //Move 120 Inches

        //init up
        waitForStart();

        sleep((long)delay);
        //play down
        telemetry.addData("Waiting", "...");
        telemetry.update();
        //sleep(5000);
        if(foundation == YES) {
            claw(OPEN);
            hook(UP);
            capServo(DOWN);
            slideTime(0.5, 3000, UP);
            drive(0.4, 10, STRAFE_RIGHT, BREAK, false);
            drive(0.4, 31.7, BACKWARDS, BREAK, false);
            hook(DOWN);
            slideTime(0.5, 3000, DOWN);
            //sleep(300);
            drive(0.5, 36.3, FORWARD, BREAK, false);
            //sleep(3000);
            hook(UP);
            slideTime(0.5, 3000, UP);
            drive(0.5, 40, STRAFE_LEFT, BREAK, false);

            if (park == BRIDGE) {
                drive(0.7, 29, BACKWARDS, BREAK, false);
            } else if (park == WALL) {
                drive(0.7, 4, BACKWARDS,BREAK, false);
            }
            turnDegGyro(LEFT, 90, 0.6);
            monoColorLineRev(0.4, (grey), FORWARD, BLUE, 20);
            drive(0.1,0.01,BACKWARDS, BREAK, false);

        }else{
            if (park==BRIDGE){
                drive(0.7, 29, BACKWARDS,BREAK, false);
            }else{
                drive(0.7,4, BACKWARDS,BREAK, false);
            }

            turnDegGyro(LEFT, 90, 0.6);
            monoColorLineRev(0.6,grey*0.9,FORWARD, BLUE, 42);
            drive(0.1,0.01,BACKWARDS, BREAK, false);

        }
        //sleep(3000);

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








        // Step 1: Move Forward 30.25 Inches

        // Step 2: Grab The Foundation
        // Step 3: Pull The Foundation Back In To The Depot 29 Inches
        // Step 4: Strafe Left 31 Inches
        //
    }
    /*public void drive(double distance, int direction) {

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);






        double power = 0.8;
        double lfPower = 0;
        double rfPower = 0;
        double lbPower = 0;
        double rbPower = 0;

        switch(direction){
            case FORWARD:
                lf.setTargetPosition((int) (COUNTS_PER_INCH * -distance)); //distance needs to be in inches
                rf.setTargetPosition((int) (COUNTS_PER_INCH * -distance)); //For some reason, going foward gives negative encoder values
                lb.setTargetPosition((int) (COUNTS_PER_INCH * -distance));
                rb.setTargetPosition((int) (COUNTS_PER_INCH * -distance));

                lfPower = power;
                rfPower = power;
                lbPower = power;
                rbPower = power;
                break;


            case BACKWARDS:
                lf.setTargetPosition((int) (COUNTS_PER_INCH * distance)); //distance needs to be in inches
                rf.setTargetPosition((int) (COUNTS_PER_INCH * distance));
                lb.setTargetPosition((int) (COUNTS_PER_INCH * distance));
                rb.setTargetPosition((int) (COUNTS_PER_INCH * distance));
                lfPower = -power;
                rfPower = -power;
                lbPower = -power;
                rbPower = -power;
                break;
            case STRAFE_RIGHT:
                lf.setTargetPosition((int) (COUNTS_PER_INCH * -distance)); //distance needs to be in inches
                rf.setTargetPosition((int) (COUNTS_PER_INCH * distance));
                lb.setTargetPosition((int) (COUNTS_PER_INCH * distance));
                rb.setTargetPosition((int) (COUNTS_PER_INCH * -distance));

                lfPower = power;
                rfPower = -power;
                lbPower = -power;
                rbPower = power;
                break;
            case STRAFE_LEFT:
                lf.setTargetPosition((int) (COUNTS_PER_INCH * distance)); //distance needs to be in inches
                rf.setTargetPosition((int) (COUNTS_PER_INCH * -distance));
                lb.setTargetPosition((int) (COUNTS_PER_INCH * -distance));
                rb.setTargetPosition((int) (COUNTS_PER_INCH * distance));

                lfPower = -power;
                rfPower = power;
                lbPower = power;
                rbPower = -power;
                break;
        }



        lf.setPower(lfPower);
        rf.setPower(rfPower);
        lb.setPower(lbPower);
        rb.setPower(rbPower);

        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive()&& (lf.isBusy() || lb.isBusy() || rb.isBusy() || rf.isBusy())) {
            idle();
            telemetry.addData("rb",rb.getCurrentPosition());
            telemetry.addData("rf",rf.getCurrentPosition());
            telemetry.addData("lf",lf.getCurrentPosition());
            telemetry.addData("lb",lb.getCurrentPosition());
            telemetry.update();
        }
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void freeform(int distance, int angle){ //angle in radians
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setTargetPosition((int) (COUNTS_PER_INCH * distance)); //distance needs to be in inches
        rf.setTargetPosition((int) (COUNTS_PER_INCH * distance));
        lb.setTargetPosition((int) (COUNTS_PER_INCH * distance));
        rb.setTargetPosition((int) (COUNTS_PER_INCH * distance));



        double lfPower = 0;
        double rfPower = 0;
        double lbPower = 0;
        double rbPower = 0;

        //Trig
        //double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        //returns hypotonuse (C value in triangle)

        double robotAngle = angle - Math.PI / 4;
        //return angle x (next to center of circle)

        //double rightX = -gamepad1.right_stick_x;
        //rotiation

        final double lfPow =  Math.sin(angle);
        final double rfPow =  Math.cos(angle);
        final double lbPow =  Math.cos(angle);
        final double rbPow =  Math.sin(angle);
        //determines wheel power

        lf.setPower(lfPower);
        rf.setPower(rfPower);
        lb.setPower(lbPower);
        rb.setPower(rbPower);

        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }





    public void hook(int direction){
        switch (direction) {
            case UP:
                hook.setPosition(.9);
                sleep(1200);
                break;
            case DOWN:
                hook.setPosition(0.15);
                sleep(1200);
                break;

        }



    }*/

    //Tiles are 24x24 inches

}