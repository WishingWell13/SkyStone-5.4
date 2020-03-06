package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Test_Strafe") //Driver controlled
public class Test_Strafe  extends Test_Abstract{

    public DcMotor lf, rf, lb, rb, ls; //Define Motors In Code

    @Override
    public void init() {
        lf = hardwareMap.dcMotor.get("lf");
        rf = hardwareMap.dcMotor.get("rf");
        lb = hardwareMap.dcMotor.get("lb");
        rb = hardwareMap.dcMotor.get("rb");

        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if(gamepad2.x){
            lf.setPower(0.8);
            rf.setPower(-0.8);
            lb.setPower(-0.8);
            rb.setPower(0.8);
        }else if(gamepad2.b){
            lf.setPower(-0.8);
            rf.setPower(0.8);
            lb.setPower(0.8);
            rb.setPower(-0.8);
        }else{
            lf.setPower(0);
            rf.setPower(0);
            lb.setPower(0);
            rb.setPower(0);
        }
    }
}
