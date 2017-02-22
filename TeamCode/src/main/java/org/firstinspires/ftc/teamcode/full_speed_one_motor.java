package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

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
@Autonomous(name = "full_speed_one_motor", group = "Concept")
//@Disabled
public class full_speed_one_motor extends LinearOpMode {

    static final int    CYCLE_MS    =   50;     // period of each cycle
    // Define class members
    private DcMotor vacuumMotor = null;

    double  power   = 0.5;

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode(){

        // Connect to motor (Assume standard left wheel)
        // Change the text in quotes to match any motor name on your robot.
        vacuumMotor = hardwareMap.dcMotor.get("motor");

        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        //leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        vacuumMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
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

            // Set the motor to the new power and pause;
            vacuumMotor.setPower(power);
            sleep(CYCLE_MS);
            idle();
        }

        // Turn off motor and signal done;
        vacuumMotor.setPower(0);

        telemetry.addData(">", "Done");
        telemetry.update();

    }
}
