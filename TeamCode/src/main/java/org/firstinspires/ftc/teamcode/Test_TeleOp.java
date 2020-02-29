package org.firstinspires.ftc.teamcode;


//imports:
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "Meet_3_TeleOp") //Driver controlled
public class Test_TeleOp extends Test_Abstract{
    //Variable: Define all servos, motors, sensors
    DcMotor lf, rf, lb, rb, ls; //Define Motors In Code
    public Gamepad g1, g2;
    Servo clawL, clawR;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        telemetry.addData("Status", "Waiting!!");


        //Motor Define In Phone
        initStuff();



    }

    @Override
    public void loop() {

        Suit_Trig();
        claw();
        hook();
        //incrementBlock();

        //Linear Slide
        linearSlide();
        telemetry.update();

    }

  /*  public class JoystickCalc
    {
        private OpMode opmode;

        double leftStickY;
        double leftStickX;
        double rightStickX;
        double rightStickY;
        boolean xButton;
        boolean yButton;
        boolean aButton;
        boolean bButton;

        public JoystickCalc(OpMode opmode)
        {
            this.opmode = opmode;
        }

        public void calculate ()
        {
            leftStickY = opmode.gamepad1.left_stick_y;
            leftStickX = opmode.gamepad1.left_stick_x;
            rightStickX = opmode.gamepad1.right_stick_x;
            rightStickY = opmode.gamepad1.right_stick_y;
            xButton = opmode.gamepad1.x;
            yButton = opmode.gamepad1.y;
            bButton = opmode.gamepad1.b;
            aButton = opmode.gamepad1.a;
        }*/
}

