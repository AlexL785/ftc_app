/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
@TeleOp(name = "Autonomous", group = "Autonomous")
//@Disabled
public class Autonomous extends LinearOpMode {

    ColorSensor colorSensor;    // Hardware Device Object

    ModernRoboticsI2cRangeSensor rangeSensor;

    Servo servo_beacon;
    private DcMotor leftMotorF = null;
    private DcMotor leftMotorB = null;
    private DcMotor rightMotorF = null;
    private DcMotor rightMotorB = null;

    final double PERFECT_COLOR_VALUE = 0.04;
    OpticalDistanceSensor lightSensor;

    double powerLeftMotorB = 0;
    double powerLeftMotorF = 0;
    double powerRightMotorB = 0;
    double powerRightMotorF = 0;

    double beacon_left_pos = 0;
    double beacon_right_pos = 1;
    double dist_init = 5;
    double dist_fin = 2;
    double motor_power = 0.4;

    boolean team_red = false;  /// TEAM COLOR
    boolean finish = false;

    void forward(double power){
        leftMotorF.setPower(power);
        leftMotorB.setPower(power);
        rightMotorF.setPower(power);
        rightMotorB.setPower(power);
    }

    void follow(){
        double power_follow = -0.2;
        while(true){
            telemetry.addData("ODS", "follow");
            double correction = (PERFECT_COLOR_VALUE - lightSensor.getLightDetected()) + 0.1;

            // Sets the powers so they are no less than .075 and apply to correction
            if (correction <= 0) {
                powerLeftMotorB = power_follow - correction;
                powerLeftMotorF = power_follow - correction;
                powerRightMotorB = power_follow;
                powerRightMotorF = power_follow;
                } else {
                powerLeftMotorB = power_follow;
                powerLeftMotorF = power_follow;
                powerRightMotorB = power_follow + correction;
                powerRightMotorF = power_follow + correction;
                }
            // Sets the powers to the motors
            leftMotorB.setPower(powerLeftMotorB);
            leftMotorB.setPower(powerLeftMotorF);
            rightMotorB.setPower(powerRightMotorB);
            rightMotorB.setPower(powerRightMotorF);
            if(rangeSensor.getDistance(DistanceUnit.CM) <= dist_init){
                break;
            }
        }
    }

    @Override
    public void runOpMode() {


        servo_beacon = hardwareMap.servo.get("beacon");
        colorSensor = hardwareMap.colorSensor.get("sensor_color");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");
        lightSensor = hardwareMap.opticalDistanceSensor.get("ods");
        leftMotorF = hardwareMap.dcMotor.get("left_drive_front");
        leftMotorB = hardwareMap.dcMotor.get("left_drive_back");
        rightMotorF = hardwareMap.dcMotor.get("right_drive_front");
        rightMotorB = hardwareMap.dcMotor.get("right_drive_back");

        ///   initialise motors

        leftMotorF.setDirection(DcMotor.Direction.REVERSE);
        leftMotorB.setDirection(DcMotor.Direction.REVERSE);
        rightMotorF.setDirection(DcMotor.Direction.FORWARD);
        rightMotorB.setDirection(DcMotor.Direction.FORWARD);

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;

        // bLedOn represents the state of the LED.
        boolean bLedOn = false;

        // get a reference to our ColorSensor object.


        // Set the LED in the beginning
        //colorSensor.enableLed(bLedOn);
        colorSensor.enableLed(bLedOn);

        // wait for the start button to be pressed.
        waitForStart();

        // while the op mode is active, loop and read the RGB data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.


        while (opModeIsActive() && !finish) {

            // check the status of the x button on either gamepad.
            bCurrState = gamepad1.x;



            // check for button state transitions.
            if ((bCurrState == true) && (bCurrState != bPrevState))  {

                // button is transitioning to a pressed state. So Toggle LED
                bLedOn = !bLedOn;
                colorSensor.enableLed(bLedOn);
            }



            // update previous state variable.
            bPrevState = bCurrState;

            // convert the RGB values to HSV values.
            double red = colorSensor.red();
            double green = colorSensor.green();
            double blue = colorSensor.blue();

            //forward(-motor_power);

            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

            /// verify distance from the beacon

            follow();

            if(lightSensor.getLightDetected() == PERFECT_COLOR_VALUE){
                forward(0);
                telemetry.addData("ODS", "found");
                follow();
            }

            if(rangeSensor.getDistance(DistanceUnit.CM) <= dist_init){

                if(red > blue && red > green && team_red){
                    servo_beacon.setPosition(beacon_left_pos);
                } else if(blue > red && blue > green && !team_red){
                    servo_beacon.setPosition(beacon_right_pos);
                }

                while(rangeSensor.getDistance(DistanceUnit.CM) > dist_fin){

                    forward(motor_power);
                }
                forward(-motor_power);
                finish = true;
                break;
            }

            // send the info back to driver station using telemetry function.
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.addData("ODS", lightSensor.getLightDetected());

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            telemetry.update();
        }
    }
}
