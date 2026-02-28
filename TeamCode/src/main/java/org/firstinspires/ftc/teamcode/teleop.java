/*
 * Copyright (c) 2025 FIRST
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import static org.firstinspires.ftc.teamcode.teleop.LaunchState.LAUNCHING;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™. It leverages a differential/Skid-Steer
 * system for robot mobility, one high-speed motor driving two "launcher wheels", and two servos
 * which feed that launcher.
 *
 * Likely the most niche concept we'll use in this example is closed-loop motor velocity control.
 * This control method reads the current speed as reported by the motor's encoder and applies a varying
 * amount of power to reach, and then hold a target velocity. The FTC SDK calls this control method
 * "RUN_USING_ENCODER". This contrasts to the default "RUN_WITHOUT_ENCODER" where you control the power
 * applied to the motor directly.
 * Since the dynamics of a launcher wheel system varies greatly from those of most other FTC mechanisms,
 * we will also need to adjust the "PIDF" coefficients with some that are a better fit for our application.
 */
@TeleOp(name = "FinalTeleOp", group = "FinalCode")
//@Disabled
public class teleop extends OpMode {
    final double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */
    final double LAUNCHER_TARGET_VELOCITY = 1500;
    final double LAUNCHER_MIN_VELOCITY = 1400;
    final double INTAKE_TARGET_VELOCITY = 6767;


    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private boolean intakeOn = false;
    private boolean Outtake = false;
    private boolean prevIntakeButtonX = false;
    private boolean prevOutTakeButtonA = false;
    final double INTAKE_POWER = 0.9;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    private CRServo claw = null;
    private boolean clawOpen = false;
    private DcMotor intake = null;
    private DcMotorEx intakeFlywheel = null;
    private int shotsFired = 0;

    ElapsedTime feederTimer = new ElapsedTime();

    /*
     * TECH TIP: State Machines
     * We use a "state machine" to control our launcher motor and feeder servos in this program.
     * The first step of a state machine is creating an enum that captures the different "states"
     * that our code can be in.
     * The core advantage of a state machine is that it allows us to continue to loop through all
     * of our code while only running specific code when it's necessary. We can continuously check
     * what "State" our machine is in, run the associated code, and when we are done with that step
     * move on to the next state.
     * This enum is called the "LaunchState". It reflects the current condition of the shooter
     * motor and we move through the enum when the user asks our code to fire a shot.
     * It starts at idle, when the user requests a launch, we enter SPIN_UP where we get the
     * motor up to speed, once it meets a minimum speed then it starts and then ends the launch process.
     * We can use higher level code to cycle through these states. But this allows us to write
     * functions and autonomous routines in a way that avoids loops within loops, and "waits".
     */
    enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    private LaunchState launchState;

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftFrontPower;
    double rightFrontPower;
    public enum StartingSide { RED, BLUE }
    StartingSide side = StartingSide.RED;
    double leftBackPower;
    double rightBackPower;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        launchState = LaunchState.IDLE;

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftRear");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightRear");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        intake = hardwareMap.get(DcMotor.class, "intake");
        intakeFlywheel = hardwareMap.get(DcMotorEx.class, "wheelIntake");
        

        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Tiggerbot Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
            if (gamepad1.dpad_left) clawOpen = true;
            if (gamepad1.dpad_right) clawOpen = false;


            telemetry.addData("ClawOpen = ", clawOpen);
            telemetry.update();

    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        boolean currentIntakeButtonX = gamepad1.x || gamepad2.x;
        boolean OuttakeButtonA = gamepad1.a || gamepad2.a;

        // Check for a rising edge (just pressed)
        if (currentIntakeButtonX && !prevIntakeButtonX) { // 
            // Flip the state of the intake
            intakeOn = !intakeOn;
        }
        if (OuttakeButtonA && !prevOutTakeButtonA) {
            Outtake = !Outtake;

        }


        // Apply power based on the current state
        if (intakeOn) {
            intake.setPower(-INTAKE_POWER);
        } else {
            intake.setPower(STOP_SPEED);
        }
        if (Outtake) {
            intake.setPower(INTAKE_POWER);
        } else {
            intake.setPower(STOP_SPEED);
        }



        /*
         * Here we call a function called arcadeDrive. The arcadeDrive function takes the input from
         * the joysticks, and applies power to the left and right drive motor to move the robot
         * as requested by the driver. "arcade" refers to the control style we're using here.
         * Much like a classic arcade game, when you move the left joystick forward both motors
         * work to drive the robot forward, and when you move the right joystick left and right
         * both motors work to rotate the robot. Combinations of these inputs can be used to create
         * more complex maneuvers.
         */
        double forward = 1;
        mecanumDrive(-gamepad1.left_stick_y * forward, gamepad1.left_stick_x * forward, gamepad1.right_stick_x * forward);

        /*
         * Here we give the user control of the speed of the launcher motor without automatically
         * queuing a shot.
         */
        //hihihihi
        if (gamepad1.right_bumper) { // spin up flywheel
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else if (gamepad1.b) { // stop flywheel
            launcher.setVelocity(0);
        } else if (gamepad1.y) {
            intakeFlywheel.setVelocity(120);
        }
        /*
        if (gamepad1.dpad_down){
            claw.setPower(FULL_SPEED);
            clawOpen = true;
        } else if (gamepad1.dpad_right) {
            claw.setPower(STOP_SPEED);
            clawOpen = false;
        }
        */

        /*
         * Now we call our "Launch" function.
         */
        launch(gamepad1.x);

        /*
         * Show the state and motor powers
         */


        telemetry.addData("Status", "Running");
        telemetry.addData("Bot Statistics", "");
        telemetry.addData("Launcher State", launchState);
        telemetry.addData("Intake State", intakeOn ? "ON" : "OFF");
        telemetry.addData("OuttakeState", Outtake ? "ON" : "OFF");
        telemetry.addData("Flywheel Speed", launcher.getVelocity());
        telemetry.addData("Intake Power", intake.getPower());
        telemetry.addData("Left Front Power", leftFrontPower);
        telemetry.addData("Right Front Power", rightFrontPower);
        telemetry.addData("Left Back Power", leftBackPower);
        telemetry.addData("Right Back Power", rightBackPower);
        telemetry.addData("Claw State", clawOpen ? "OPEN" : "CLOSED");
        telemetry.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    void mecanumDrive(double forward, double strafe, double rotate) {

        /* the denominator is the largest motor power (absolute value) or 1
         * This ensures all the powers maintain the same ratio,
         * but only if at least one is out of the range [-1, 1]
         */
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFrontPower = (forward + strafe + rotate) / denominator;
        rightFrontPower = (forward - strafe - rotate) / denominator;
        leftBackPower = (forward - strafe + rotate) / denominator;
        rightBackPower = (forward + strafe - rotate) / denominator;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

    }

    void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    shotsFired = 0;
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    feederTimer.reset();
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH: {
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                launchState = LaunchState.LAUNCHING;
            }
            //hi

            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                }
                break;
        }
    }
}































































/*
 hey man
 you.
 YOU.
 YoU.
 YOU have been digging through our files.
 dont copy us
 unless we said so
 or we will know
 and we will come for you
 im in ur walls
 AI is stupid
 ishan is short
 ishansped
 Cr7 is better
 't listen yot laksh
 messi is the goat

 kpop demon hunters is goated
 JINU JINU JINU JINU JINU JINU JINU JINU JINU
 NO ITS NOT
 DONT
 NO
 JINU SUCKS
  GFDRYTGIUOPYIFTF
  PWFH[

  frhpi
  gp
  ogpgrpgpgdp
  hdhtihipgdrh
  pjpg
  pjgrj
  gjpgdrj
  gdj
  ogjoppgjdopjodgjpogd
  GO PROGRAM PRANAV
  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7  CR7
M10 MESSI MESSI MESSI MESSI MESSI MESSI MESSI GOAT m
 */