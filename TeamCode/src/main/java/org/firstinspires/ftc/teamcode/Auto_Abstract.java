package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
//import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//imports


public abstract class Auto_Abstract extends LinearOpMode {
    //variables
    private DcMotor lf, rf, lb, rb, ls;
    //public Gamepad g1, g2;
    private Servo clawL, clawR, hook, capServo;
    private ColorSensor colorSensor, skyStoneColor;    // Hardware Device Object


    ColorSensor colorRev, colorLineRev;
    private DistanceSensor sensorDistance/*, distanceLineRev*/;
    private BNO055IMU imu;

    //NormalizedRGBA colors = colorRev.getNormalizedColors();
    private float[] hsvValues = new float[3];
    //private final float values[] = hsvValues;
    //float hsvRev[] = {0F, 0F, 0F};
    //final float valuesRev[] = hsvValues;
    private final double SCALE_FACTOR = 255;





    private ElapsedTime runtime = new ElapsedTime();
    //Directions
    private static final int FORWARD = 0;
    private static final int BACKWARDS = 1;
    private static final int STRAFE_RIGHT = 2;
    private static final int STRAFE_LEFT = 3;
    //static final int FREEFORM = 4;
    private static final int UP = 0;
    private static final int DOWN = 1;
    static final int CLOSE = 0;
    static final int OPEN = 1;
    static final int PART = 2;
    private static final int SUP_PART = 3;

    static final int RED = 0;
    private static final int GREEN = 1;
    static final int BLUE = 2;
    private static final int LUM = 3;

    private static final int WALL = 0;
    private static final int BRIDGE = 1;

    static final int YES = 0;
    private static final int NO = 1;

    private double gray = 0;
    private double yellow = 0;

    //Gyro Setup
    private Orientation lastAngles = new Orientation();
    private double globalAngle = .30;

    //private double startOrient = 0;


    static boolean LEFT = false;
    static boolean RIGHT = true;

    static boolean GLIDE = true;
    static boolean BREAK = false;





    //Encoder Setup
    static final
    double PI = Math.PI;
    private static final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: TETRIX Motor Encoder
    private static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 4;     // For figuring circumference
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * PI);
    //initStuff (claw, hook, wheels)

    //other functions
    /* todo: Reduce safety stop so robot stops on Red Line
       Fix Auto-correct strafe drive auto
       Implement delay for autonomi





     */
    /*Think about:
     * One autonomous for each half, then button press into the halves
     * f
     * */




    public double getGrayLum(){
        gray = 0;
        for (int i = 1; i <= 300; i ++){
            gray += colorSensor.alpha();
        }
        gray /= 300;
        return gray;
    }

    double getGrayBlue(){
        gray = 0;
        for (int i = 1; i <= 300; i ++){
            gray += colorSensor.blue();
        }
        gray /= 300;
        return gray;
    }
    double getGrayRed(){
        gray = 0;
        for (int i = 1; i <= 300; i ++){
            gray += colorSensor.red();
        }
        gray /= 300;
        return gray;
    }

    public double getGrayLumRev(){
        gray = 0;
        for (int i = 1; i <= 300; i ++){
            gray += colorLineRev.alpha();
        }
        gray /= 300;
        return gray;
    }

    public double getGrayBlueRev(){
        gray = 0;
        for (int i = 1; i <= 300; i ++){
            gray += colorLineRev.blue();
        }
        gray /= 300;
        return gray;
    }
    double getGrayRedRev(){
        gray = 0;
        for (int i = 1; i <= 300; i ++){
            gray += colorLineRev.red();
        }
        gray /= 300;
        return gray;
    }

    public double getBlockLum(){
        yellow = 0;
        for (int i = 1; i <= 300; i ++){
            yellow += colorRev.alpha();
        }
        yellow /= 300;
        return yellow;
    }

    public double getBlockRed(){
        yellow = 0;
        for (int i = 1; i <= 300; i ++){
            yellow += colorRev.alpha();
        }
        yellow /= 300;
        return yellow;
    }

    private double getHeadingRad()
    {
        //get's current angle/way you are facing
        final Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return angles.firstAngle;
    }

    private double getHeadingDeg(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public int buttonsPark(){
        telemetry.addLine("Parking Side? (Wait 1 Second)");
        telemetry.addLine("A:Wall Side");
        telemetry.addLine("B: Bridge Side");
        telemetry.update();
        sleep(1000);

        while ((!gamepad1.a && !gamepad1.b) && !isStopRequested()){
            if (gamepad1.a){
                telemetry.addLine("Press Recieved");
                telemetry.update();
                sleep(1000);
                return WALL;
            }else if (gamepad1.b){
                telemetry.addLine("Press Recieved");
                telemetry.update();
                sleep(1000);
                return BRIDGE;
            }
        }
        telemetry.addLine("Press Not Recieved");
        telemetry.update();
        sleep(1000);
        return -1;
    }

    int buttonsParkv2(){
        telemetry.addLine("Parking Side? (Wait 1 Second)");
        telemetry.addLine("A:Wall Side");
        telemetry.addLine("B: Bridge Side");
        telemetry.update();
        sleep(1000);
        while ((!gamepad1.a && !gamepad1.b) && !isStopRequested()){
            idle();
        }
        //while ((!gamepad1.a && !gamepad1.b) && !isStopRequested()){
        if (gamepad1.a){
            telemetry.addLine("Press Recieved");
            telemetry.update();
            sleep(1000);
            return WALL;
        } else if (gamepad1.b){
            telemetry.addLine("Press Recieved");
            telemetry.update();
            sleep(1000);
            return BRIDGE;
        }
        //}
        telemetry.addLine("Press Not Recieved");
        telemetry.update();
        sleep(1000);
        return -1;
    }

    int buttonSample(){
        telemetry.addLine("Sample? (Wait 1 Second)");
        telemetry.addLine("A: Yes");
        telemetry.addLine("B: No");
        telemetry.update();
        sleep(1000);
        while ((!gamepad1.a && !gamepad1.b) && !isStopRequested()){
            idle();
        }
        if (gamepad1.a){
            telemetry.addLine("Press Recieved");
            telemetry.update();
            sleep(1000);
            return YES;
        }else if (gamepad1.b){
            telemetry.addLine("Press Recieved");
            telemetry.update();
            sleep(1000);
            return NO;
        }

        telemetry.addLine("Press Not Recieved");
        telemetry.update();
        sleep(1000);
        return -1;
    }

    int buttonFoundation(){
        boolean pressed = false;
        telemetry.addLine("Foundation? (Wait 1 Second)");
        telemetry.addLine("A: Yes");
        telemetry.addLine("B: No");
        telemetry.update();
        sleep(1000);
        while ((!gamepad1.a && !gamepad1.b) && !isStopRequested()){
            while (!pressed) {
                if (gamepad1.a) {
                    telemetry.addLine("Press Recieved");
                    telemetry.update();
                    sleep(1000);
                    pressed=true;
                    return YES;
                } else if (gamepad1.b) {
                    telemetry.addLine("Press Recieved");
                    telemetry.update();
                    sleep(1000);
                    pressed=true;
                    return NO;
                }
            }
        }
        telemetry.addLine("Press Not Recieved");
        telemetry.update();
        sleep(1000);
        return -1;

    }

    double buttonDelay(){  //returns delay in miliseconds
        boolean done = false;
        double delay = 0;
        boolean aPressed = false; //is a pressed?
        boolean bPressed = false;
        boolean xPressed = false;
        boolean yPressed = false;

        while(!done && !isStopRequested()){
            telemetry.addLine("Press A to add 5 seconds");  //Telemetry for drivers
            telemetry.addLine("Press B to add 1 second");
            telemetry.addLine("Press X to reset timer");
            telemetry.addLine("Press Y to continue");
            telemetry.addData("Current Delay: ", delay);
            telemetry.addLine(" milliseconds");
            telemetry.update();

            if(gamepad1.a && aPressed ==false){ //Checks to see if button wasn't previously pressed
                delay += 5000;
                aPressed=true;
            }
            if (gamepad1.b && bPressed ==false){
                delay += 1000;
                bPressed=true;
            }
            if (gamepad1.x && xPressed ==false){
                delay = 0;
                xPressed=true;
            }
            if (gamepad1.y && yPressed ==false){
                done=true;
                yPressed=true;
            }

            if(!gamepad1.a){ //Resets button booleans once button is released
                aPressed=false;
            }
            if(!gamepad1.b){
                bPressed=false;
            }
            if(!gamepad1.x){
                xPressed=false;
            }
            if(!gamepad1.y){
                yPressed=false;
            }

        }
        telemetry.addLine("Press Received");
        telemetry.update();
        return delay;
    }

    public int buttonSkystonePos(){
        telemetry.addLine("Wall Stone Or Bridge Stone (Wait 1 Second)");
        telemetry.addLine("A: Wall");
        telemetry.addLine("B: Bridge");
        telemetry.update();
        sleep(1000);
        while ((!gamepad1.a && !gamepad1.b) && !isStopRequested()){
            idle();
        }
        if (gamepad1.a){
            telemetry.addLine("Press Recieved (WALL)");
            telemetry.update();
            sleep(1000);
            return WALL;
        }else if (gamepad1.b){
            telemetry.addLine("Press Recieved");
            telemetry.update();
            sleep(1000);
            return BRIDGE;
        }
        telemetry.addLine("Press Not Recieved");
        return -1;

    }

    public int buttonSenseSkystone(){
        telemetry.addLine("Grab Stone Without Sensor?");
        telemetry.addLine("A: Yes");
        telemetry.addLine("B: No");
        telemetry.update();
        sleep(1000);
        while ((!gamepad1.a && !gamepad1.b) && !isStopRequested()){
            idle();
        }
        if (gamepad1.a){
            telemetry.addLine("Press Recieved");
            telemetry.update();
            sleep(1000);
            return YES;
        }else if (gamepad1.b){
            telemetry.addLine("Press Recieved");
            telemetry.update();
            sleep(1000);
            return NO;
        }
        telemetry.addLine("Press Not Recieved");
        return -1;

    }

    public void generalDefine(){
        //Motor Define
        boolean bLedOn = true;
        lf = hardwareMap.dcMotor.get("lf");
        rf = hardwareMap.dcMotor.get("rf");
        lb = hardwareMap.dcMotor.get("lb");
        rb = hardwareMap.dcMotor.get("rb");
        ls = hardwareMap.dcMotor.get("ls");
        //Servo Define
        clawL = hardwareMap.servo.get("clawL");
        clawR = hardwareMap.servo.get("clawR");
        capServo = hardwareMap.servo.get("capper");
        hook = hardwareMap.servo.get("hook");

        colorSensor = hardwareMap.get(ColorSensor.class, "color");
        skyStoneColor = hardwareMap.get(ColorSensor.class, "SkyColor");
        // get a reference to the color sensor.
        colorRev = hardwareMap.get(ColorSensor.class, "colorRev");
        colorLineRev = hardwareMap.get(ColorSensor.class, "colorLine");
        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "colorRev");

        colorSensor.enableLed(bLedOn);
        skyStoneColor.enableLed(bLedOn);
        if (colorRev instanceof SwitchableLight) {
            ((SwitchableLight)colorRev).enableLight(true);
        }

        //------------------------------------------------------------------------------------------------------//
        //Gyro Stuff
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        //--------------------------------------------------------------------------------------------------------//
        //color_sensor = hardwareMap.colorSensor.get("color");

        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);


        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ls.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Encoder Stuff
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //----------------------------------------------------------------------------------------------------------------//
        //Foundation Only
        //Move 120 Inches
    }

    public void slideTime(double power, int time, int direction){
        if(direction == UP){
            ls.setPower(power);
        }else{
            ls.setPower(-power);
        }
        sleep(time);
    }
    static int countMotorsBusy(DcMotor... motors) {
        int count = 0;
        for(final DcMotor motor : motors) {
            if (motor.isBusy()) {
                ++count;
            }
        }
        return count;
    }

    public void drive(double power, double distance, int direction, boolean glide, boolean part) { //parameters: When you use the function, the code will ask for these two variables

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (glide){
            lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }else{
            lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        double sPower = 0.4;
        double lfPower = 0;
        double rfPower = 0;
        double lbPower = 0;
        double rbPower = 0;

        double slowDist = 4 * COUNTS_PER_INCH;

        if (!part){
            distance += (slowDist/COUNTS_PER_INCH);
        }

        switch(direction){
            case FORWARD:
                lf.setTargetPosition((int) ((COUNTS_PER_INCH * -distance) + slowDist)); //distance needs to be in inches
                rf.setTargetPosition((int) (COUNTS_PER_INCH * -distance + slowDist)); //For some reason, going forward gives negative encoder values
                lb.setTargetPosition((int) (COUNTS_PER_INCH * -distance + slowDist));
                rb.setTargetPosition((int) (COUNTS_PER_INCH * -distance + slowDist));

                lfPower = power;
                rfPower = power;
                lbPower = power;
                rbPower = power;
                break;

            case BACKWARDS:
                lf.setTargetPosition((int) (COUNTS_PER_INCH * distance- slowDist)); //distance needs to be in inches
                rf.setTargetPosition((int) (COUNTS_PER_INCH * distance- slowDist));
                lb.setTargetPosition((int) (COUNTS_PER_INCH * distance- slowDist));
                rb.setTargetPosition((int) (COUNTS_PER_INCH * distance- slowDist));
                lfPower = -power;
                rfPower = -power;
                lbPower = -power;
                rbPower = -power;
                break;
            case STRAFE_RIGHT: // What is case?
                lf.setTargetPosition((int) (COUNTS_PER_INCH * -distance + slowDist)); //distance needs to be in inches
                rf.setTargetPosition((int) (COUNTS_PER_INCH * distance- slowDist));
                lb.setTargetPosition((int) (COUNTS_PER_INCH * distance- slowDist));
                rb.setTargetPosition((int) (COUNTS_PER_INCH * -distance + slowDist));

                lfPower = power;
                rfPower = -power;
                lbPower = -power;
                rbPower = power;



                break;
            case STRAFE_LEFT:
                lf.setTargetPosition((int) (COUNTS_PER_INCH * distance - slowDist)); //distance needs to be in inches
                rf.setTargetPosition((int) (COUNTS_PER_INCH * -distance + slowDist));
                lb.setTargetPosition((int) (COUNTS_PER_INCH * -distance +slowDist));
                rb.setTargetPosition((int) (COUNTS_PER_INCH * distance- slowDist));

                lfPower = -power;
                rfPower = power;
                lbPower = power;
                rbPower = -power;
                break;
        }

        //Sign of setPower does not matter for RUN_TO_POSITION

        lf.setPower(lfPower);
        rf.setPower(rfPower);
        lb.setPower(lbPower);
        rb.setPower(rbPower);

        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addLine("****begining loop****1");
        telemetry.update();
        //sleep(1000);

        while (opModeIsActive()&& (countMotorsBusy(lf,lb,rf,rb) > 3)) {
            idle();
            telemetry.addData("rb",rb.getCurrentPosition());
            telemetry.addData("rf",rf.getCurrentPosition());
            telemetry.addData("lf",lf.getCurrentPosition());
            telemetry.addData("lb",lb.getCurrentPosition());
            telemetry.update();
        }

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);

        telemetry.addLine("****ended loop****1");
        telemetry.update();
        //sleep(1000);
        //Phase 2 ---------------------------------------------
        if (part) {

            power /= 2;

            switch (direction) {
                case FORWARD:
                    lf.setTargetPosition((int) (-slowDist)); //distance needs to be in inches
                    rf.setTargetPosition((int) (-slowDist)); //For some reason, going forward gives negative encoder values
                    lb.setTargetPosition((int) (-slowDist));
                    rb.setTargetPosition((int) (-slowDist));


                    lfPower = power;
                    rfPower = power;
                    lbPower = power;
                    rbPower = power;
                    break;

                case BACKWARDS:
                    lf.setTargetPosition((int) (slowDist)); //distance needs to be in inches
                    rf.setTargetPosition((int) (slowDist));
                    lb.setTargetPosition((int) (slowDist));
                    rb.setTargetPosition((int) (slowDist));
                    lfPower = -power;
                    rfPower = -power;
                    lbPower = -power;
                    rbPower = -power;
                    break;
                case STRAFE_RIGHT: // What is case?
                    lf.setTargetPosition((int) (-slowDist)); //distance needs to be in inches
                    rf.setTargetPosition((int) (slowDist));
                    lb.setTargetPosition((int) (slowDist));
                    rb.setTargetPosition((int) (-slowDist));

                    lfPower = power;
                    rfPower = -power;
                    lbPower = -power;
                    rbPower = power;


                    break;
                case STRAFE_LEFT:
                    lf.setTargetPosition((int) (slowDist)); //distance needs to be in inches
                    rf.setTargetPosition((int) (-slowDist));
                    lb.setTargetPosition((int) (-slowDist));
                    rb.setTargetPosition((int) (slowDist));

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

            telemetry.addLine("****begining loop****2");
            telemetry.update();
            //sleep(1000);

            while (opModeIsActive() && (countMotorsBusy(lf,lb,rf,rb) > 3)) {
                idle();
                telemetry.addData("rb", rb.getCurrentPosition());
                telemetry.addData("rf", rf.getCurrentPosition());
                telemetry.addData("lf", lf.getCurrentPosition());
                telemetry.addData("lb", lb.getCurrentPosition());
                telemetry.addData("rb Tar", rb.getTargetPosition());
                telemetry.addData("rf Tar", rf.getTargetPosition());
                telemetry.addData("lf Tar", lf.getTargetPosition());
                telemetry.addData("lb Tar", lb.getTargetPosition());
                telemetry.addLine("Part 2 Running");
                telemetry.update();
            }
            telemetry.addLine("****Ending loop****2");
            telemetry.update();
            //sleep(1000);
        }


        //End Phase 2 -----------------------------------------
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }



    public void encoderTurn180(double power){
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        double sPower = 0.4;
        double lfPower = 0;
        double rfPower = 0;
        double lbPower = 0;
        double rbPower = 0;

        lf.setTargetPosition(-1725); //distance needs to be in inches
        rf.setTargetPosition(1612);  // What is setTargetPosition?
        lb.setTargetPosition(-1723); // It tells the wheel how far to go.
        rb.setTargetPosition(1613);
        lfPower = power;
        rfPower = -power;
        lbPower = power;
        rbPower = -power;

        lf.setPower(lfPower);
        rf.setPower(rfPower);
        lb.setPower(lbPower);
        rb.setPower(rbPower);

        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive()&& countMotorsBusy(lf, lb, rf, rb) > 1) {
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


    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public void turnDegGyro(boolean dir, int deg, double power){
        telemetry.addData("Doing", "turnDeg");
        telemetry.update();
        resetAngle();

        //left is true
        double lpwr = dir ? -power : power; // if true: if false
        //ternary operators
        double rpwr = -lpwr;
        lf.setPower(lpwr);
        rf.setPower(rpwr);
        lb.setPower(lpwr);
        rb.setPower(rpwr);//just setPower

        while((Math.abs(getHeadingDeg()) < Math.abs(deg) - 20) && opModeIsActive()){
            telemetry.addData("Angle:", getHeadingDeg());
            telemetry.update();
            idle();
        }
        lpwr /= 4;
        rpwr /= 4;
        lf.setPower(lpwr);
        rf.setPower(rpwr);
        lb.setPower(lpwr);
        rb.setPower(rpwr);
        while(Math.abs(getHeadingDeg()) < 20 && opModeIsActive()){
            telemetry.addData("Angle 2:", getHeadingDeg());
            telemetry.update();
            idle();
        }

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);


    }


    //------------Drive Colors ----------------------Drive Colors---------------------------Drive Colors-------------
    public void monoColorDrive(double power, double color, int direction, int target, int errorE) { //parameters: When you use the function, the code will ask for these two variables

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);







        double sPower = 0.4;
        double lfPower = 0;
        double rfPower = 0;
        double lbPower = 0;
        double rbPower = 0;

        switch(direction){
            case FORWARD:


                lfPower = -power;
                rfPower = -power;
                lbPower = -power;
                rbPower = -power;
                break;


            case BACKWARDS:

                lfPower = power;
                rfPower = power;
                lbPower = power;
                rbPower = power;
                break;
            case STRAFE_RIGHT:


                lfPower = -power;
                rfPower = power;
                lbPower = power;
                rbPower = -power;

                break;
            case STRAFE_LEFT:

                lfPower = power;
                rfPower = -power;
                lbPower = -power;
                rbPower = power;
                break;
        }



        lf.setPower(lfPower);
        rf.setPower(rfPower);
        lb.setPower(lbPower);
        rb.setPower(rbPower);

        int error = 42;

        switch (target) {
            case RED:
                while (opModeIsActive() && (colorSensor.red() < color + (0.5 * (color+0.01)))&&
                        (Math.abs(rb.getCurrentPosition()) <= (errorE*COUNTS_PER_INCH)) &&
                        (Math.abs(rf.getCurrentPosition()) <= (errorE*COUNTS_PER_INCH))&&
                        (Math.abs(lb.getCurrentPosition()) <= (errorE*COUNTS_PER_INCH)) &&
                        (Math.abs(lf.getCurrentPosition()) <= (errorE*COUNTS_PER_INCH))) {
                    idle();
                    telemetry.addData("Red  ", colorSensor.red());
                    telemetry.addData("lf", lf.getCurrentPosition());
                    telemetry.update();
                }
                break;
            case GREEN:
                while (opModeIsActive() && (colorSensor.green() < color + (0.5 * (color+0.01)))
                        &&
                        (rb.getCurrentPosition() < (errorE*COUNTS_PER_INCH)) &&
                        (rf.getCurrentPosition() < (errorE*COUNTS_PER_INCH))&&
                        (lb.getCurrentPosition() <= (errorE*COUNTS_PER_INCH)) &&
                        (lf.getCurrentPosition() < (errorE*COUNTS_PER_INCH))){
                    idle();
                    telemetry.addData("Green", colorSensor.green());
                }
                break;
            case BLUE:
                while (opModeIsActive() && (colorSensor.blue() < color + (0.5 * (color+0.01))) &&
                        (Math.abs(rb.getCurrentPosition()) <= (errorE*COUNTS_PER_INCH)) &&
                        (Math.abs(rf.getCurrentPosition()) <= (errorE*COUNTS_PER_INCH))&&
                        (Math.abs(lb.getCurrentPosition()) <= (errorE*COUNTS_PER_INCH)) &&
                        (Math.abs(lf.getCurrentPosition()) <= (errorE*COUNTS_PER_INCH))) {
                    idle();
                    telemetry.addData("Blue ", colorSensor.blue());
                }
                break;
            case LUM:
                while (opModeIsActive() && (colorSensor.alpha() < color + color + (0.5 * (color+0.01))) &&
                        (rb.getCurrentPosition() < (errorE*COUNTS_PER_INCH)) &&
                        (rf.getCurrentPosition() < (errorE*COUNTS_PER_INCH))&&
                        (lb.getCurrentPosition() <= (errorE*COUNTS_PER_INCH)) &&
                        (lf.getCurrentPosition() < (errorE*COUNTS_PER_INCH))) {
                    idle();
                    telemetry.addData("Clear", colorSensor.alpha());
                }

        }

        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);


    }
    public void monoColorLineRev(double power, double color, int direction, int target, int errorE) { //parameters: When you use the function, the code will ask for these two variables

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);







        double sPower = 0.4;
        double lfPower = 0;
        double rfPower = 0;
        double lbPower = 0;
        double rbPower = 0;

        switch(direction){
            case FORWARD:


                lfPower = -power;
                rfPower = -power;
                lbPower = -power;
                rbPower = -power;
                break;


            case BACKWARDS:

                lfPower = power;
                rfPower = power;
                lbPower = power;
                rbPower = power;
                break;
            case STRAFE_RIGHT:


                lfPower = -power;
                rfPower = power;
                lbPower = power;
                rbPower = -power;

                break;
            case STRAFE_LEFT:

                lfPower = power;
                rfPower = -power;
                lbPower = -power;
                rbPower = power;
                break;
        }



        lf.setPower(lfPower);
        rf.setPower(rfPower);
        lb.setPower(lbPower);
        rb.setPower(rbPower);

        int error = 42;

        Color.RGBToHSV((int) (colorLineRev.red() * SCALE_FACTOR),
                (int) (colorLineRev.green() * SCALE_FACTOR),
                (int) (colorLineRev.blue() * SCALE_FACTOR),
                hsvValues);

        switch (target) {
            case RED:
                while (opModeIsActive() && (colorLineRev.red() < color + (0.2 * (color))&&
                        (Math.abs(rb.getCurrentPosition()) <= (errorE*COUNTS_PER_INCH)) &&
                        (Math.abs(rf.getCurrentPosition()) <= (errorE*COUNTS_PER_INCH))&&
                        (Math.abs(lb.getCurrentPosition()) <= (errorE*COUNTS_PER_INCH)) &&
                        (Math.abs(lf.getCurrentPosition()) <= (errorE*COUNTS_PER_INCH)))) {
                    idle();
                    telemetry.addData("Red  ", colorSensor.red());
                    telemetry.addData("lf", lf.getCurrentPosition());
                    telemetry.update();
                }
                break;
            case GREEN:
                while (opModeIsActive() && (colorLineRev.green() < color + (0.2 * (color)))
                        &&
                        (rb.getCurrentPosition() < (errorE*COUNTS_PER_INCH)) &&
                        (rf.getCurrentPosition() < (errorE*COUNTS_PER_INCH))&&
                        (lb.getCurrentPosition() <= (errorE*COUNTS_PER_INCH)) &&
                        (lf.getCurrentPosition() < (errorE*COUNTS_PER_INCH))){
                    idle();
                    telemetry.addData("Green", colorSensor.green());
                }
                break;
            case BLUE:
                while (opModeIsActive() && (colorLineRev.blue() < color + (0.2 * (color))) &&
                        (Math.abs(rb.getCurrentPosition()) <= (errorE*COUNTS_PER_INCH)) &&
                        (Math.abs(rf.getCurrentPosition()) <= (errorE*COUNTS_PER_INCH))&&
                        (Math.abs(lb.getCurrentPosition()) <= (errorE*COUNTS_PER_INCH)) &&
                        (Math.abs(lf.getCurrentPosition()) <= (errorE*COUNTS_PER_INCH))) {
                    idle();
                    telemetry.addData("Blue ", colorSensor.blue());
                }
                break;
            case LUM:
                while (opModeIsActive() && (colorLineRev.alpha() < color *1.2) &&
                        (rb.getCurrentPosition() < (errorE*COUNTS_PER_INCH)) &&
                        (rf.getCurrentPosition() < (errorE*COUNTS_PER_INCH))&&
                        (lb.getCurrentPosition() <= (errorE*COUNTS_PER_INCH)) &&
                        (lf.getCurrentPosition() < (errorE*COUNTS_PER_INCH))) {
                    idle();
                    telemetry.addData("Clear", colorSensor.alpha());
                }

        }

        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);


    }

    public void monoColorSkyRev(double power, double color, int direction, int target, int errorE) { //parameters: When you use the function, the code will ask for these two variables

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Color.RGBToHSV((int) (colorRev.red() * SCALE_FACTOR),
                (int) (colorRev.green() * SCALE_FACTOR),
                (int) (colorRev.blue() * SCALE_FACTOR),
                hsvValues);





        double sPower = 0.4;
        double lfPower = 0;
        double rfPower = 0;
        double lbPower = 0;
        double rbPower = 0;

        switch(direction){
            case FORWARD:


                lfPower = power;
                rfPower = power;
                lbPower = power;
                rbPower = power;
                break;


            case BACKWARDS:

                lfPower = -power;
                rfPower = -power;
                lbPower = -power;
                rbPower = -power;
                break;
            case STRAFE_RIGHT:


                lfPower = -power;
                rfPower = power;
                lbPower = power;
                rbPower = -power;
                break;
            case STRAFE_LEFT:

                lfPower = power;
                rfPower = -power;
                lbPower = -power;
                rbPower = power;
                break;
        }



        lf.setPower(lfPower);
        rf.setPower(rfPower);
        lb.setPower(lbPower);
        rb.setPower(rbPower);

        //NormalizedRGBA colors = colorRev.getNormalizedColors(); //Ask NormalizedColor VS Color

        //Yellow Block Red Reading: 191
        //Yellow Block Green reading: 300
        //No Block Red Reading: 118
        //Skystone Red Reading: 112

        switch (target) {
            case RED:
                while (opModeIsActive() && (colorRev.red() >= color - (0.04 * (color+0.1)))
                        &&
                        (rb.getCurrentPosition() <= (errorE*COUNTS_PER_INCH)) &&
                        (rf.getCurrentPosition() <= (errorE*COUNTS_PER_INCH))&&
                        (lb.getCurrentPosition() <= (errorE*COUNTS_PER_INCH)) &&
                        (lf.getCurrentPosition() <= (errorE*COUNTS_PER_INCH)))  {
                    idle();
                    telemetry.addData("Red  ", colorRev.red());
                }
                break;
            case GREEN:
                while (opModeIsActive() && (colorRev.green() >= color - (0.04 * (color+0.1)))
                        &&
                        (rb.getCurrentPosition() <= (errorE*COUNTS_PER_INCH)) &&
                        (rf.getCurrentPosition() <= (errorE*COUNTS_PER_INCH))&&
                        (lb.getCurrentPosition() <= (errorE*COUNTS_PER_INCH)) &&
                        (lf.getCurrentPosition() <= (errorE*COUNTS_PER_INCH))) {
                    idle();
                    telemetry.addData("Green", colorRev.green());
                }
                break;
            case BLUE:
                while (opModeIsActive() && (colorRev.blue() >= color - (0.04 * (color+0.1)))
                        &&
                        (rb.getCurrentPosition() <= (errorE*COUNTS_PER_INCH)) &&
                        (rf.getCurrentPosition() <= (errorE*COUNTS_PER_INCH))&&
                        (lb.getCurrentPosition() <= (errorE*COUNTS_PER_INCH)) &&
                        (lf.getCurrentPosition() <= (errorE*COUNTS_PER_INCH))) {
                    idle();
                    telemetry.addData("Blue ", colorRev.blue());
                    /*while (sensorDistance.getDistance(DistanceUnit.CM) >= errorR){
                        drive(0.4, 0.1,direction);
                    }*/
                }
                break;
            case LUM:
                while (opModeIsActive() && (colorRev.alpha() >= color - (0.04 * (color+0.1)))
                        &&
                        (rb.getCurrentPosition() <= (errorE*COUNTS_PER_INCH)) &&
                        (rf.getCurrentPosition() <= (errorE*COUNTS_PER_INCH))&&
                        (lb.getCurrentPosition() <= (errorE*COUNTS_PER_INCH)) &&
                        (lf.getCurrentPosition() <= (errorE*COUNTS_PER_INCH))) {
                    idle();
                    telemetry.addData("Clear", colorRev.alpha());
                    /*while (sensorDistance.getDistance(DistanceUnit.CM) >= errorR){
                        drive(0.4, 0.1,direction);
                    }*/
                }

        }

        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);


    }

    public void monoColorDriveSky(double power, double color, int direction, int target, int errorE) { //parameters: When you use the function, the code will ask for these two variables
        //for Modern Robotics Color Sensor
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);







        double sPower = 0.4;
        double lfPower = 0;
        double rfPower = 0;
        double lbPower = 0;
        double rbPower = 0;

        switch(direction){
            case FORWARD:


                lfPower = power;
                rfPower = power;
                lbPower = power;
                rbPower = power;
                break;


            case BACKWARDS:

                lfPower = -power;
                rfPower = -power;
                lbPower = -power;
                rbPower = -power;
                break;
            case STRAFE_RIGHT:

                lfPower = -power;
                rfPower = power;
                lbPower = power;
                rbPower = -power;


                break;
            case STRAFE_LEFT:

                lfPower = power;
                rfPower = -power;
                lbPower = -power;
                rbPower = power;
                break;
        }



        lf.setPower(lfPower);
        rf.setPower(rfPower);
        lb.setPower(lbPower);
        rb.setPower(rbPower);


        switch (target) {
            case RED:
                while (opModeIsActive() && (skyStoneColor.red() >= color) &&
                        (rb.getCurrentPosition() <= (36*COUNTS_PER_INCH)) &&
                        (rf.getCurrentPosition() <= (36*COUNTS_PER_INCH))&&
                        (lb.getCurrentPosition() <= (36*COUNTS_PER_INCH)) &&
                        (lf.getCurrentPosition() <= (36*COUNTS_PER_INCH))){
                    idle();
                    telemetry.addData("Red Sky ", skyStoneColor.red());
                }
                break;
            case GREEN:
                while (opModeIsActive() && (skyStoneColor.green() >= color)&&
                        (rb.getCurrentPosition() < (36*COUNTS_PER_INCH)) &&
                        (rf.getCurrentPosition() < (36*COUNTS_PER_INCH))&&
                        (lb.getCurrentPosition() <= (36*COUNTS_PER_INCH)) &&
                        (lf.getCurrentPosition() < (36*COUNTS_PER_INCH))) {
                    idle();
                    telemetry.addData("Green Sky ", skyStoneColor.green());
                }
                break;
            case BLUE:
                while (opModeIsActive() && (skyStoneColor.blue() >= color)&&
                        (rb.getCurrentPosition() < (36*COUNTS_PER_INCH)) &&
                        (rf.getCurrentPosition() < (36*COUNTS_PER_INCH))&&
                        (lb.getCurrentPosition() <= (36*COUNTS_PER_INCH)) &&
                        (lf.getCurrentPosition() < (36*COUNTS_PER_INCH))) {
                    idle();
                    telemetry.addData("Blue Sky ", skyStoneColor.blue());
                }
                break;
            case LUM:
                while (opModeIsActive() && (skyStoneColor.alpha() >= color)&&
                        (rb.getCurrentPosition() < (36*COUNTS_PER_INCH)) &&
                        (rf.getCurrentPosition() < (36*COUNTS_PER_INCH))&&
                        (lb.getCurrentPosition() <= (36*COUNTS_PER_INCH)) &&
                        (lf.getCurrentPosition() < (36*COUNTS_PER_INCH))) {
                    idle();
                    telemetry.addData("Clear Sky ", skyStoneColor.alpha());
                }

        }

        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);


    }

    public void driveRange(double power, double distance, int direction, int error){

    }



    //----------Other--------------------Other---------------Other
    public void hook(int direction){
        switch (direction) {
            case UP:
                hook.setPosition(.9);
                //sleep(1200);
                break;
            case DOWN:
                hook.setPosition(0.15);
                //sleep(1200);
                break;
        }
    }

    public void capServo(int state){
        if(state == UP) {
            capServo.setPosition(1);
        }
        else {
            capServo.setPosition(0);
        }
    }

    public void claw(int state){
        switch (state){
            case CLOSE:
                clawR.setPosition(.7); // close claw
                clawL.setPosition(.20);
                sleep(500);
                break;
            case OPEN:
                clawR.setPosition(.13); //open claw
                clawL.setPosition(.8);
                sleep(500);
                break;
            case PART:
                clawR.setPosition(.55); //partial claw
                clawL.setPosition(.4);
                sleep(500);
                break;
            case SUP_PART:
                clawR.setPosition(.53); //partial claw
                clawL.setPosition(.42);
                sleep(200);
        }

    }

    private double getHeading()
    {
        //get's current angle/way you are facing
        final Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return angles.firstAngle;

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


        double startOrient = 0;
        double lfPower = 0;
        double rfPower = 0;
        double lbPower = 0;
        double rbPower = 0;



        final double currentHeading = getHeading();


        double angleDiff = startOrient - currentHeading;


        /*
        Add Holonomic Drive Components
        Need separate mechanism from encoders (wheels move different amounts when move other than

         */



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





}