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
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
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

/*

    * Code by purplecoder

 */
@TeleOp(name = "Autonomous_beta", group = "Autonomous")
//@Disabled
public class Autonomous_beta extends LinearOpMode {

    ColorSensor colorSensor;    // Hardware Device Object

    ModernRoboticsI2cRangeSensor rangeSensor;

    Servo servo_beacon;
    private DcMotor leftMotorF = null;
    private DcMotor leftMotorB = null;
    private DcMotor rightMotorF = null;
    private DcMotor rightMotorB = null;

    private DcMotor throwMotor = null;

    Servo servo_box;
    Servo servo_selector;

    final double PERFECT_COLOR_VALUE = 0.04;
    ModernRoboticsI2cGyro gyro;
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

    double throwPower = 1;

    double box_down_pos = 0.0;
    double box_up_pos = 0.5;
    double selector_up_pos = 0.4;
    double selector_down_pos = 0.0;

    boolean team_red = false;  /// TEAM COLOR
    boolean finish = false;

    int heading;

    int distance = 1575; //adymark 1120

    void forward(double power){
        leftMotorF.setPower(power);
        leftMotorB.setPower(power);
        rightMotorF.setPower(power);
        rightMotorB.setPower(power);
    }

    void rotate(double power){
        leftMotorB.setPower(power);
        leftMotorF.setPower(power);
        rightMotorB.setPower(-power);
        rightMotorF.setPower(-power);
    }

    void follow(){
        double power_follow = -0.2;
        while(true){
            telemetry.addData("ODS", "follow");
            double correction = (PERFECT_COLOR_VALUE - lightSensor.getLightDetected()) * 5;

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

    void capture_beacon(){

        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);

        double red = colorSensor.red();
        double green = colorSensor.green();
        double blue = colorSensor.blue();

        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        if(red > blue && red > green && team_red){
            servo_beacon.setPosition(beacon_left_pos);
        } else if(blue > red && blue > green && !team_red){
            servo_beacon.setPosition(beacon_right_pos);
        }

        while(rangeSensor.getDistance(DistanceUnit.CM) > dist_fin){
            forward(motor_power);
        }
        sleep(500);
        forward(-motor_power);
        sleep(500);
    }

    ///
    void rotate_gyro(int degree, int stanga){
        int fin_state = (gyro.getHeading() + degree) % 360;
        while(gyro.getHeading() != fin_state){
            rotate(motor_power * stanga);
        }
        forward(0);
        sleep(500);
    }

    public void turnPosition(double power){


        throwMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        throwMotor.setTargetPosition(-distance);

        throwMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turn(power);


        while(throwMotor.isBusy()){
            //wait until target position is reached
        }

        turn(0);
        throwMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turn(double power){
        throwMotor.setPower(power);
    }

    public void load(){
        servo_box.setPosition(box_down_pos);

        servo_selector.setPosition(selector_down_pos);
    }


    @Override
    public void runOpMode() {

        ModernRoboticsI2cGyro gyro;   // Hardware Device Object
        int xVal, yVal, zVal = 0;     // Gyro rate Values
        //int heading = 0;              // Gyro integrated heading
        int angleZ = 0;
        boolean lastResetState = false;
        boolean curResetState  = false;

        // get a reference to a Modern Robotics GyroSensor object.
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        // start calibrating the gyro.
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        gyro.calibrate();

        // make sure the gyro is calibrated.
        while (!isStopRequested() && gyro.isCalibrating())  {
            sleep(50);
            idle();
        }


        servo_beacon = hardwareMap.servo.get("beacon");
        colorSensor = hardwareMap.colorSensor.get("sensor_color");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");
        lightSensor = hardwareMap.opticalDistanceSensor.get("ods");
        leftMotorF = hardwareMap.dcMotor.get("left_drive_front");
        leftMotorB = hardwareMap.dcMotor.get("left_drive_back");
        rightMotorF = hardwareMap.dcMotor.get("right_drive_front");
        rightMotorB = hardwareMap.dcMotor.get("right_drive_back");
        throwMotor = hardwareMap.dcMotor.get("throw");

        servo_box = hardwareMap.servo.get("box");
        servo_selector = hardwareMap.servo.get("selector");

        ///   initialise motors

        leftMotorF.setDirection(DcMotor.Direction.REVERSE);
        leftMotorB.setDirection(DcMotor.Direction.REVERSE);
        rightMotorF.setDirection(DcMotor.Direction.FORWARD);
        rightMotorB.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        turnPosition(throwPower);  //throw one particle
        sleep(2000);

        load();

        turnPosition(throwPower); //throw one particle
        sleep(2000);

        rotate_gyro(45, -1);
        sleep(2000);

        forward(motor_power);
        sleep(1500);
        forward(0);

        follow();

        capture_beacon();

        rotate_gyro(270, 1);

        forward(motor_power);
        sleep(1000);
        forward(0);

        rotate_gyro(90, -1);

        follow();

        capture_beacon();

        rotate_gyro(45, -1);

        forward(-motor_power);
        sleep(2000);
        forward(0);



        /*
        while (opModeIsActive() && !finish) {


            follow();

            if(lightSensor.getLightDetected() == PERFECT_COLOR_VALUE){
                forward(0);
                telemetry.addData("ODS", "found");
                follow();
            }

            if(rangeSensor.getDistance(DistanceUnit.CM) <= dist_init){


                finish = true;
                break;
            }

            // send the info back to driver station using telemetry function.
            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            //telemetry.addData("Hue", hsvValues[0]);
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
        */
    }
}
