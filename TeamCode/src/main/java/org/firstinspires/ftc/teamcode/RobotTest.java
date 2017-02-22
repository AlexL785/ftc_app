package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
/**
 * Created by DAN on 09.12.2016.
 */
@TeleOp(name="Robot_Test", group="Concept")
//@Disabled
public class RobotTest extends LinearOpMode {

    //static final double MAX_FWD     =  1.0;     // Maximum FWD power applied to motor
    //static final double MAX_REV     = -1.0;     // Maximum REV power applied to motor

    int ANDYMARK_TICKS_PER_REV = 1120; //ticks per revolution for andymark motor

    // Define class members
    private DcMotor throwMotor = null;
    private DcMotor leftMotorF = null;
    private DcMotor leftMotorB = null;
    private DcMotor rightMotorF = null;
    private DcMotor rightMotorB = null;

    static final int    CYCLE_MS    =   50;
    long pauseTime = 3000;
    int distance = 1580 ;
    boolean rampUp  = true;

    //private ElapsedTime runtime = new ElapsedTime();

    public void strafeLeft() {
        leftMotorB.setPower(1.0);
        rightMotorF.setPower(1.0);
    }

    public void strafeRight() {
        leftMotorF.setPower(1.0);
        rightMotorB.setPower(1.0);
    }

    public void turnPosition(double power){


        throwMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        throwMotor.setTargetPosition(distance);

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





    @Override
    public void runOpMode() throws InterruptedException {

        double throwPower = 1;
        double rightWheelPowerF = 0.0;
        double rightWheelPowerB = 0.0;
        double leftWheelPowerF = 0.0;
        double leftWheelPowerB = 0.0;
        double max1 = 0.0;
        double max2 = 0.0;

        // Connect to motor (Assume standard left wheel)
        // Change the text in quotes to match any motor name on your robot.
        throwMotor = hardwareMap.dcMotor.get("throw");
        leftMotorF = hardwareMap.dcMotor.get("left_drive_front");
        leftMotorB = hardwareMap.dcMotor.get("left_drive_back");
        rightMotorF = hardwareMap.dcMotor.get("right_drive_front");
        rightMotorB = hardwareMap.dcMotor.get("right_drive_back");


        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        leftMotorF.setDirection(DcMotor.Direction.FORWARD);
        leftMotorB.setDirection(DcMotor.Direction.FORWARD);
        rightMotorF.setDirection(DcMotor.Direction.REVERSE);
        rightMotorB.setDirection(DcMotor.Direction.REVERSE);
        throwMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        throwMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Say", "Hello Driver");
        telemetry.addData("Status", "Initialized");
        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motor.");
        telemetry.update();

        waitForStart();

        // Ramp motor speeds till stop pressed.
        while (opModeIsActive()) {
/*
            if(gamepad1.dpad_left)
                strafeLeft();
            else if(gamepad1.dpad_right)
                strafeRight();
            else {
                leftMotorF.setPower(0);
                leftMotorB.setPower(0);
                rightMotorF.setPower(0);
                rightMotorB.setPower(0);
            }
*/
            leftWheelPowerF = -gamepad1.left_stick_y;
            rightWheelPowerF = -gamepad1.right_stick_y;

            leftWheelPowerB = -gamepad1.left_stick_y;
            rightWheelPowerB = -gamepad1.right_stick_y;

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


            leftMotorF.setPower(leftWheelPowerF);
            leftMotorB.setPower(leftWheelPowerB);
            rightMotorF.setPower(rightWheelPowerF);
            rightMotorB.setPower(rightWheelPowerB);

            if (gamepad1.a)
                turnPosition(throwPower);

            // Display the current value
            telemetry.addData("Motor Power", "%5.2f", throwPower);
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();

            idle();

            sleep(CYCLE_MS);
        }

        // Turn off motor and signal done;
        throwMotor.setPower(0);
        leftMotorF.setPower(0);
        leftMotorB.setPower(0);
        rightMotorF.setPower(0);
        rightMotorB.setPower(0);

        telemetry.addData(">", "Done");
        telemetry.update();


    }
}
