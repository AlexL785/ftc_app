package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;
//Acesta e un test

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
@Autonomous(name = "ConceptTest", group = "Concept")
//@Disabled
public class Test2OpMode extends LinearOpMode {

    static final long    CYCLE_MS    =   50;     // period of each cycle
    // Define class members
    private DcMotor leftMotorF = null;
    private DcMotor leftMotorB = null;
    private DcMotor rightMotorF = null;
    private DcMotor rightMotorB = null;
    private DcMotor throwMotor = null;
    double  power   = 0.0;
    double throwPower = 0.5;
    long elapsedTime = 0;


    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Connect to motor (Assume standard left wheel)
        // Change the text in quotes to match any motor name on your robot.
        leftMotorB = hardwareMap.dcMotor.get("motf2");
        leftMotorF = hardwareMap.dcMotor.get("mot1");
        rightMotorB = hardwareMap.dcMotor.get("motf1");
        rightMotorF = hardwareMap.dcMotor.get("mot2");
        throwMotor = hardwareMap.dcMotor.get("aruncare");

        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        //leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftMotorF.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        //leftMotor1.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftMotorB.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        //rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightMotorF.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        //rightMotor1.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightMotorB.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        throwMotor.setDirection(DcMotor.Direction.FORWARD);
        telemetry.addData("Status", "Initialized");
        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motors." );
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Ramp motor speeds till stop pressed.
        while(opModeIsActive()) {

            // Display the current value
            telemetry.addData("Motor Power", "%5.2f", power);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

           throwMotor.setPower(throwPower);

            elapsedTime = 0;
            while(elapsedTime < 500000) {
                throwMotor.setPower(0.0);
                elapsedTime++;
            }

            sleep(CYCLE_MS);
            idle();
        }

        // Turn off motor and signal done;
        leftMotorF.setPower(0);
        leftMotorB.setPower(0);
        rightMotorF.setPower(0);
        rightMotorB.setPower(0);
        throwMotor.setPower(0);
        telemetry.addData(">", "Done");
        telemetry.update();

    }
}