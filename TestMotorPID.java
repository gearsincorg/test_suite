/* Copyright (c) 2018 Phil Malone. All rights reserved.
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
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.test_suite;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;


enum TestModes {
    OLP_ProportionalSpeeds,
    OLP_BrakingBehavior,
    OLP_DynamicSpeedChange,
    OLP_SmoothTransitions,

    CLV_ProportionalSpeeds,
    CLV_BrakingBehavior,
    CLV_DynamicSpeedChange,
    CLV_SmoothTransitions;

    private static TestModes[] vals = values();
    public TestModes next() { return vals[(this.ordinal()+1) % vals.length];}
    public TestModes prev() { return vals[(this.ordinal()-1+vals.length) % vals.length];}
}


/**
 * This file contains a suite of tests to validate several motor control capabilities.
 *
 * Start the opmode and select run.
 *   Then, using the gamepad Y amd A button, select the desired test.
 *   Once selected, use the B button to start the test.
 *   Most tests can be terminated by hitting the X button.
 *
 * Most tests only require a single motor to be attached.  It is named left_drive
 * An encoder MUST be attached to the motor (unless testing loss of encoder)
 *
 * Currently PIDF coefficients are preset by the opmode, for Andymark Neverest20/40/60
 */

@TeleOp(name="Motor and PID test", group="Phil Malone")
public class TestMotorPID extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime     = new ElapsedTime();
    private DcMotorEx   motor       = null;
    private TestModes   testMode    = TestModes.OLP_ProportionalSpeeds;

    // Currently required for PID testing.
    PIDCoefficients velPID = new PIDCoefficients(0.0425, 0.00425,  0.425);  // P D F  AM 40
    PIDCoefficients posPID = new PIDCoefficients(15.0,    0.0 ,     0.0 );  // P


    @Override
    public void runOpMode() {

        motor = hardwareMap.get(DcMotorEx.class, "left_drive");  // Use default motor names to use existing Configs
        motor.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, velPID);
        motor.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION,   posPID);

        waitForStart();

        // run desired test unless stop was pressed.
        while (opModeIsActive()) {

            switch (runMenuLoop()) {
                default:
                case OLP_ProportionalSpeeds:
                    OLPCLV_ProportionalSpeeds(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    break;

                case OLP_BrakingBehavior:
                    OLPCLV_BrakingBehavior(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    break;

                case OLP_DynamicSpeedChange:
                    OLPCLV_DynamicSpeedChange(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    break;

                case OLP_SmoothTransitions:
                    OLPCLV_SmoothTransitions(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    break;

                case CLV_ProportionalSpeeds:
                    OLPCLV_ProportionalSpeeds(DcMotor.RunMode.RUN_USING_ENCODER);
                    break;

                case CLV_BrakingBehavior:
                    OLPCLV_BrakingBehavior(DcMotor.RunMode.RUN_USING_ENCODER);
                    break;

                case CLV_DynamicSpeedChange:
                    OLPCLV_DynamicSpeedChange(DcMotor.RunMode.RUN_USING_ENCODER);
                    break;

                case CLV_SmoothTransitions:
                    OLPCLV_SmoothTransitions(DcMotor.RunMode.RUN_USING_ENCODER);
                    break;

            }
        }

    }

    // ==================================================================
    // Run the motor at POWERS ranging from 100% to -100% in 10% steps
    // Record the average speed and present as a percentage of max speed.
    private void OLPCLV_ProportionalSpeeds(DcMotor.RunMode mode) {
        double  speedStepSize = 0.1;
        int     speedSteps    = (int)(2.0 / speedStepSize) + 1;
        int     speedStep;
        double  reqPower;
        int     startPos;
        int     travel;

        double []  powers = new double[speedSteps];
        double []  speeds = new double[speedSteps];

        motor.setPower(0);
        motor.setMode(mode);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        speedStep = 0;
        reqPower  = 1.0;
        while (opModeIsActive() && (!gamepad1.x) && (reqPower >= -1.0)) {

            // Set the desired speed/power and then wait a second before measuring speed
            motor.setPower(reqPower);
            sleep(1000);

            runtime.reset();
            startPos  = motor.getCurrentPosition();
            travel    = 0;

            // Run for 5 seconds measuring average speed.
            while (opModeIsActive() && (!gamepad1.x) && (runtime.time() < 5.0)) {
                travel = motor.getCurrentPosition() - startPos;

                telemetry.addData(">", "%4.2f E:%05d, T:%5.3f, CPS:%4.0f",
                                  reqPower,  travel, runtime.time(), travel/runtime.time());
                telemetry.update();
            }

            // Save the results for this speed/power
            powers[speedStep] = reqPower;
            speeds[speedStep] = travel/runtime.time();

            // prepare for the next speed.
            speedStep++;
            reqPower -= speedStepSize;
        }
        motor.setPower(0.0);

        // Report results as absolute numbers and percentages.
        telemetry.addData("Help", "Press X to exit test");
        telemetry.addData(">", "Power    +cps   +%      -cps   -%");
        int lastIndex = powers.length - 1 ;
        for (int i = 0; i <= (powers.length / 2) ; i++) {
            telemetry.addData(">", "P=%04.2f %04.0f %03.0f%% %05.0f %03.0f%%",
                                powers[i],
                                speeds[i],     (speeds[i]    / speeds[0])  * 100.0,
                                speeds[lastIndex-i],  (speeds[lastIndex-i] / speeds[lastIndex]) * 100.0 );
        }
        telemetry.update();
        sleep(1000);
        while (!gamepad1.x);
    }

    // ==================================================================
    // Stop the motor and enable the user to switch between brake and float
    // Press and hold A on the gamepad to brake
    private void OLPCLV_BrakingBehavior(DcMotor.RunMode mode) {

        motor.setPower(0);
        motor.setMode(mode);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        while (opModeIsActive() && (!gamepad1.x)) {
            telemetry.addData("Help", "Press A to apply brake.");
            telemetry.addData("Help", "Press X to exit test");
            if (gamepad1.a) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                telemetry.addData("Motor", "BRAKE");
            }
            else {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                telemetry.addData("Motor", "FLOAT");
            }
            telemetry.update();
         }
    }

    // ==================================================================
    // Step the motor between 0 and selected speed (full range, 20% steps).
    // Verify single direction of travel. (no rollback)
    // Display number of rollback detections for each speed.
    private void OLPCLV_SmoothTransitions(DcMotor.RunMode mode) {
        double  speedStepSize = 0.2;
        int     speedSteps    = (int)(2.0 / speedStepSize) + 1;
        int     speedStep;
        double  reqPower;
        int     lastPos;
        int     thisPos;
        int     travel;
        int     errorCount;

        double []  powers = new double[speedSteps];
        int    []  errors = new int[speedSteps];

        motor.setPower(0);
        motor.setMode(mode);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        speedStep = 0;
        reqPower  = 1.0;
        lastPos = motor.getCurrentPosition();

        while (opModeIsActive() && !gamepad1.x && (reqPower >= -1.0)) {

            // Setup the initial conditions for this test
            errorCount  = 0;

            // Run for 1 second then stop for a second.
            // Look for any direction reversals
            runtime.reset();
            motor.setPower(reqPower);
            sleep(20);
            while (opModeIsActive() && (runtime.time() < 1.0)) {
                thisPos = motor.getCurrentPosition();;

                // Look for travel in the wrong direction.
                if (((thisPos - lastPos) * reqPower) < 0)
                    errorCount++;

                telemetry.addData(">", "%4.2f E:%05d", reqPower,  errorCount);
                telemetry.update();

                lastPos = thisPos;
                sleep(20);
            }

            runtime.reset();
            motor.setPower(0);
            while (opModeIsActive() && (runtime.time() < 1.0)) {
                thisPos = motor.getCurrentPosition();;

                // Look for travel in the wrong direction. (rollback)
                if (((thisPos - lastPos) * reqPower) < 0)
                    errorCount++;

                telemetry.addData(">", "%4.2f E:%05d", reqPower,  errorCount);
                telemetry.update();

                lastPos = thisPos;
                sleep(20);
            }

            // Save the results for this speed/power
            powers[speedStep] = reqPower;
            errors[speedStep] = errorCount;

            // prepare for the next speed.
            speedStep++;
            reqPower -= speedStepSize;
        }
        motor.setPower(0.0);

        // Report results as absolute numbers and percentages.
        telemetry.addData("Help", "Press X to exit test");
        telemetry.addData(">", "Power    +err   -err");
        int lastIndex = powers.length - 1 ;
        for (int i = 0; i <= (powers.length / 2) ; i++) {
            telemetry.addData(">", "P=%04.2f    %03d    %03d",
                    powers[i], errors[i], errors[lastIndex-i] );
        }
        telemetry.update();
        sleep(1000);
        while (!gamepad1.x);
    }

    // ==================================================================
    // Control the motor speed using the gamepad.
    // Press and hold A on the gamepad to brake
    private void OLPCLV_DynamicSpeedChange(DcMotor.RunMode mode) {
        double  reqPower;
        double  lastPos  = 1;
        double  curPos   = 0;
        double  curSpeed = 0;

        motor.setPower(0);
        motor.setMode(mode);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        runtime.reset();
        lastPos = motor.getCurrentPosition();

        while (opModeIsActive() && (!gamepad1.x)) {
            telemetry.addData("Help", "Left Joystick Y Drives");
            telemetry.addData("Help", "Press A to apply brake.");
            telemetry.addData("Help", "Press X to exit test");

            reqPower = -gamepad1.left_stick_y;
            motor.setPower(reqPower);

            if (runtime.time() > 0.25) {
                curPos = motor.getCurrentPosition();
                curSpeed = (curPos - lastPos) / runtime.time();
                runtime.reset();
                lastPos=curPos;
            }

            if (reqPower == 0.0) {
                if (gamepad1.a) {
                    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    telemetry.addData("Motor", "BRAKE Cur %4.0f CPS", curSpeed);
                }
                else {
                    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    telemetry.addData("Motor", "FLOAT Cur %4.0f CPS",  curSpeed);
                }
            }
            else {
                telemetry.addData("Motor", "Req %4.0f%% Cur %4.0f CPS", reqPower * 100, curSpeed);
            }

            telemetry.update();
        }
    }

    // ==================================================================
    // Run Menu loop
    private TestModes runMenuLoop() {

        boolean change = true;
        boolean lastUp = false;
        boolean lastDn = false;

        while (opModeIsActive() && !gamepad1.b) {

            if (change) {
                telemetry.addData("Help", "Press Y & A to choose test.");
                telemetry.addData("Help", "Press B to run test.");
                telemetry.addData(">>>", testMode);
                telemetry.update();
                change = false;
            }

            if (gamepad1.y && !lastUp) {
                testMode = testMode.prev();
                change = true;
            }

            if (gamepad1.a && !lastDn) {
                testMode = testMode.next();
                change = true;
            }

            lastUp = gamepad1.y;
            lastDn = gamepad1.a;
        }
        runtime.reset();
        return testMode;
    }
}