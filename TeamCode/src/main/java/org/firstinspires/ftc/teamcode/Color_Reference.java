package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/*
 *
 * This is an example LinearOpMode that shows how to use
 * a Modern Robotics Color Sensor.
 *
 * The op mode assumes that the color sensor
 * is configured with a name of "sensor_color".
 *
 * You can use the X button on gamepad1 to toggle the LED on and off.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "Color_Reference_N")
public class Color_Reference extends LinearOpMode{

    ColorSensor colorSensor, skyStoneColor;    // Hardware Device Object
    ColorSensor colorRev, colorLineRev;
    DistanceSensor sensorDistance, distanceLineRev;



    @Override
    public void runOpMode() {

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};
        final float hsvSky[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;
        final float sValues[] = hsvSky;

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvRev[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float valuesRev[] = hsvValues;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;

        // bLedOn represents the state of the LED.
        boolean bLedOn = true;

        colorSensor = hardwareMap.get(ColorSensor.class, "color");
        skyStoneColor = hardwareMap.get(ColorSensor.class, "SkyColor");
        // get a reference to the color sensor.
        colorRev = hardwareMap.get(ColorSensor.class, "colorRev");
        colorLineRev = hardwareMap.get(ColorSensor.class, "colorLine");
        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "colorRev");
        distanceLineRev = hardwareMap.get(DistanceSensor.class, "distanceLine");




        // Set the LED in the beginning
        colorSensor.enableLed(bLedOn);
        skyStoneColor.enableLed(bLedOn);
        final double SCALE_FACTOR = 255;


        // wait for the start button to be pressed.
        waitForStart();

        // while the op mode is active, loop and read the RGB data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {

            // check the status of the x button on either gamepad.
            bCurrState = gamepad1.x;

            // check for button state transitions.
            if (bCurrState && (bCurrState != bPrevState))  {

                // button is transitioning to a pressed state. So Toggle LED
                bLedOn = !bLedOn;
                colorSensor.enableLed(bLedOn);
            }

            // update previous state variable.
            bPrevState = bCurrState;

            // convert the RGB values to HSV values.
            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
            Color.RGBToHSV(skyStoneColor.red() * 8, skyStoneColor.green() * 8, skyStoneColor.blue() * 8, hsvSky);

            // send the info back to driver station using telemetry function.
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.addData("Hue", hsvValues[0]);

            telemetry.addData("LED Sky", bLedOn ? "On" : "Off");
            telemetry.addData("Clear Sky ", skyStoneColor.alpha());
            telemetry.addData("Red Sky ", skyStoneColor.red());
            telemetry.addData("Green Sky ", skyStoneColor.green());
            telemetry.addData("Blue Sky ", skyStoneColor.blue());
            telemetry.addData("Hue Sky", hsvSky[0]);

            Color.RGBToHSV((int) (colorRev.red() * SCALE_FACTOR),
                    (int) (colorRev.green() * SCALE_FACTOR),
                    (int) (colorRev.blue() * SCALE_FACTOR),
                    hsvValues);

            // send the info back to driver station using telemetry function.
            telemetry.addLine();
            telemetry.addLine("colorRev:");
            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("Alpha", colorRev.alpha());
            telemetry.addData("Red  ", colorRev.red());
            telemetry.addData("Green", colorRev.green());
            telemetry.addData("Blue ", colorRev.blue());
            telemetry.addData("Hue", hsvValues[0]);

            // send the info back to driver station using telemetry function.
            telemetry.addLine("Line Rev");
            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", distanceLineRev.getDistance(DistanceUnit.CM)));
            telemetry.addData("Alpha", colorLineRev.alpha());
            telemetry.addData("Red  ", colorLineRev.red());
            telemetry.addData("Green", colorLineRev.green());
            telemetry.addData("Blue ", colorLineRev.blue());
            telemetry.addData("Hue", hsvValues[0]);




            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, sValues));
                }
            });

            telemetry.update();
        }

        // Set the panel back to the default color
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });
    }
}
