package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This OpMode ramps a single motor speed up and down repeatedly until Stop is pressed.
 * The code is structured as a LinearOpMode
 *
 * This code assumes a DC motor configured with the name "left_drive" as is found on a pushbot.
 *
 * INCREMENT sets how much to increase/decrease the power each cycle
 * CYCLE_MS sets the update period.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name = "final_test", group = "Concept")
//@Disabled
public class test_final extends LinearOpMode {

    //static final double MAX_FWD     =  1.0;     // Maximum FWD power applied to motor
    //static final double MAX_REV     = -1.0;     // Maximum REV power applied to motor

    int ANDYMARK_TICKS_PER_REV = 1120; //ticks per revolution for andymark motor

    // Define class members

    private DcMotor leftMotor = null;
    private DcMotor leftMotor1 = null;
    private DcMotor rightMotor = null;
    private DcMotor rightMotor1 = null;
    private DcMotor throwMotor = null;
    double  power   = 1.0;

    double  throwPower   = 1;
    static final int    CYCLE_MS    =   50;
    long pauseTime = 3000;
    int distance = 1560 ;
    boolean rampUp  = true;

    //private ElapsedTime runtime = new ElapsedTime();

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

        // Connect to motor (Assume standard left wheel)
        // Change the text in quotes to match any motor name on your robot.
        throwMotor = hardwareMap.dcMotor.get("aruncare");
        leftMotor = hardwareMap.dcMotor.get("mot1");
        leftMotor1 = hardwareMap.dcMotor.get("motf1");
        rightMotor = hardwareMap.dcMotor.get("mot2");
        rightMotor1 = hardwareMap.dcMotor.get("motf2");


        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        //leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        //throwMotor.setDirection(DcMotor.Direction.REVERSE);
        // / Set to REVERSE if using AndyMark motors
        leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        //leftMotor1.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftMotor1.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        //rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        //rightMotor1.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightMotor1.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        throwMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        throwMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motor.");
        telemetry.update();

        int nr = 100000;
        while (nr > 0) {
            leftMotor.setPower(power);
            leftMotor1.setPower(power);
            rightMotor.setPower(power);
            rightMotor1.setPower(power);
            idle();
            nr--;
        }
        leftMotor.setPower(0);
        leftMotor1.setPower(0);
        rightMotor.setPower(0);
        rightMotor1.setPower(0);
        idle();

        waitForStart();

        // Ramp motor speeds till stop pressed.
        while(opModeIsActive()) {

            turnPosition(throwPower);

            // Display the current value
            telemetry.addData("Motor Power", "%5.2f", throwPower);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // sleep(CYCLE_MS);

            idle();

            sleep(pauseTime);
        }

        // Turn off motor and signal done;
        throwMotor.setPower(0);
        telemetry.addData(">", "Done");
        telemetry.update();

    }
}