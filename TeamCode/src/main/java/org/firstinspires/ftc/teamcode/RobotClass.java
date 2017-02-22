package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Purplecoder
 */
@TeleOp(name="Robot_Test3", group="Concept")
//@Disabled
public class RobotClass extends LinearOpMode {

    //static final double MAX_FWD     =  1.0;     // Maximum FWD power applied to motor
    //static final double MAX_REV     = -1.0;     // Maximum REV power applied to motor


    int ANDYMARK_TICKS_PER_REV = 1120; //ticks per revolution for andymark motor

    // Define class members
    // ModernRoboticsI2cRangeSensor rangeSensor ;
    private DcMotor throwMotor = null;
    private DcMotor vacuumMotor = null;
    private DcMotor glisieraMotor = null;
    private DcMotor leftMotorF = null;
    private DcMotor leftMotorB = null;
    private DcMotor rightMotorF = null;
    private DcMotor rightMotorB = null;
    Servo servo_box;
    Servo servo_selector;
    Servo servo_beacon;
    Servo servo_furca;

    TouchSensor sensor_touch;

    double glisiera_speed = 1;

    static final int    CYCLE_MS    =   50;
    static final double POWER_RAMP_DOWN = 0.2; // ramp down power
    // long pauseTime = 3000;
    int distance = 1575;
    // boolean rampUp  = true;

    //private ElapsedTime runtime = new ElapsedTime();

    public void turnPosition(double power){


        throwMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        throwMotor.setTargetPosition(-distance);

        throwMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turn(power);


        while(throwMotor.isBusy()){
            //wait until target position is reached
        }

        StopTurning();
        throwMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turn(double power){
        throwMotor.setPower(power);
    }

    public void StopTurning(){
        turn(0);
    }

    public void vacuumUp(double power) {
        vacuumMotor.setDirection(DcMotor.Direction.REVERSE);
        vacuumMotor.setPower(power);
    }

    public void vacuumDown(double power) {
        vacuumMotor.setDirection(DcMotor.Direction.FORWARD);
        vacuumMotor.setPower(power);
    }

    public boolean driver_stopped(double a, double b, double c, double d){
        if(a==0 && b==0 && c==0 && d==0) {
            return true;
        }else return false;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        double throwPower = 1;
        double vacuumPower = 0.6;
        double rightWheelPowerF = 0.0;
        double rightWheelPowerB = 0.0;
        double leftWheelPowerF = 0.0;
        double leftWheelPowerB = 0.0;
        double max1 = 0.0;
        double max2 = 0.0;
        double box_down_pos = 0.0;
        double box_up_pos = 0.5;
        double selector_up_pos = 0.4;
        double selector_down_pos = 0.0;
        double beacon_left_pos = 0.0;
        double beacon_right_pos = 1.0;

        // Connect to motor (Assume standard left wheel)
        // Change the text in quotes to match any motor name on your robot.
        throwMotor = hardwareMap.dcMotor.get("throw");
        vacuumMotor = hardwareMap.dcMotor.get("vacuum");
        glisieraMotor = hardwareMap.dcMotor.get("glisiera");
        leftMotorF = hardwareMap.dcMotor.get("left_drive_front");
        leftMotorB = hardwareMap.dcMotor.get("left_drive_back");
        rightMotorF = hardwareMap.dcMotor.get("right_drive_front");
        rightMotorB = hardwareMap.dcMotor.get("right_drive_back");
        servo_box = hardwareMap.servo.get("box");
        servo_selector = hardwareMap.servo.get("selector");
        servo_beacon = hardwareMap.servo.get("beacon");
        servo_furca = hardwareMap.servo.get("furca");
        sensor_touch = hardwareMap.touchSensor.get("touch");

        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        leftMotorF.setDirection(DcMotor.Direction.REVERSE);
        leftMotorB.setDirection(DcMotor.Direction.REVERSE);
        rightMotorF.setDirection(DcMotor.Direction.FORWARD);
        rightMotorB.setDirection(DcMotor.Direction.FORWARD);
        throwMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        throwMotor.setDirection(DcMotor.Direction.REVERSE);
        vacuumMotor.setDirection(DcMotor.Direction.FORWARD);
        glisieraMotor.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Say", "Hello Driver!");
        telemetry.addData("Status", "Initialized");
        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motor.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

                leftWheelPowerF = gamepad1.left_stick_y;
                rightWheelPowerF = gamepad1.right_stick_y;

                leftWheelPowerB = gamepad1.left_stick_y;
                rightWheelPowerB = gamepad1.right_stick_y;

                // Normalize the values so neither exceed +/- 1.0
                max1 = Math.max(Math.abs(leftWheelPowerF), Math.abs(rightWheelPowerF));
                max2 = Math.max(Math.abs(leftWheelPowerB), Math.abs(rightWheelPowerB));
                if (max1 > 1.0)
                {
                    leftWheelPowerF /= max1;
                    rightWheelPowerF /= max1;
                }
                if(max2 > 1.0) {
                    leftWheelPowerB /= max2;
                    rightWheelPowerB /= max2;
                }

            /// vacuum
            if (gamepad1.right_bumper) {
                vacuumUp(vacuumPower);
            } else if(gamepad1.left_bumper) {
                vacuumDown(vacuumPower);
            } else {
                vacuumMotor.setPower(0);
            }

            /// box
            if (gamepad2.dpad_up) {
                servo_box.setPosition(box_up_pos);
            } else if(gamepad2.dpad_down) {
                servo_box.setPosition(box_down_pos);
            }

            ///selector
            if (gamepad2.b) {
                servo_selector.setPosition(selector_up_pos);
            } else {
                servo_selector.setPosition(selector_down_pos);
            }

            /// throw
            if (gamepad2.a)
                turnPosition(throwPower);

            /// beacon
            if (gamepad1.dpad_left) {
                servo_box.setPosition(beacon_left_pos);
            } else if(gamepad1.dpad_right) {
                servo_box.setPosition(beacon_right_pos);
            }

            /// glisiera
            if (gamepad1.dpad_up) {
                glisieraMotor.setPower(glisiera_speed);
            } else if(gamepad1.dpad_down) {
                glisieraMotor.setPower(-glisiera_speed);
            } else {
                glisieraMotor.setPower(0);
            }

            /// furca
            if (gamepad1.a) {
                servo_furca.setPosition(1.0);
            } else if(gamepad1.b) {
                servo_furca.setPosition(0.0);
            } else {
                servo_furca.setPosition(0.5);
            }

            /// set power to drive motors
            leftMotorF.setPower(leftWheelPowerF);
            leftMotorB.setPower(leftWheelPowerB);
            rightMotorF.setPower(rightWheelPowerF);
            rightMotorB.setPower(rightWheelPowerB);

            // Display the current value
            telemetry.addData("left_front_motor", leftWheelPowerF);
            telemetry.addData("right_front_motor", rightWheelPowerF);
            telemetry.addData("left_back_motor", leftWheelPowerB);
            telemetry.addData("right_back_motor", rightWheelPowerB);
            telemetry.update();

            idle();

            sleep(CYCLE_MS);
        }

        // Turn off motor and signal done;
        throwMotor.setPower(0);
        vacuumMotor.setPower(0);
        leftMotorF.setPower(0);
        leftMotorB.setPower(0);
        rightMotorF.setPower(0);
        rightMotorB.setPower(0);

        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
