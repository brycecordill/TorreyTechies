package org.firstinspires.ftc.teamcode.Relic;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Bryce on 9/26/2017.
 */

@TeleOp(name = "Operation Telly", group = "3650")
public class TeleOp_3650 extends OpMode{
    DcMotor lDrive, rDrive, lift1, lift2;
    Servo armServo, grabber;

    @Override
    public void init() {
        rDrive = hardwareMap.dcMotor.get("rDrive");

        lDrive = hardwareMap.dcMotor.get("lDrive");
        lDrive.setDirection(DcMotor.Direction.REVERSE);

        lift1 = hardwareMap.dcMotor.get("lift1");
        lift2 = hardwareMap.dcMotor.get("lift2");

        armServo = hardwareMap.servo.get("armServo");
        grabber = hardwareMap.servo.get("grabber");

    }

    @Override
    public void loop() {
        lDrive.setPower(gamepad1.left_stick_y);
        rDrive.setPower(gamepad1.right_stick_y);

        // lift
        if (gamepad2.right_trigger > 0.1){
            lift1.setPower(0.7 * gamepad2.right_trigger);
            lift2.setPower(0.7 * gamepad2.right_trigger);
        }
        else if (gamepad2.left_trigger > 0.1){
            lift1.setPower(-0.7 * gamepad2.left_trigger);
            lift2.setPower(-0.7 * gamepad2.left_trigger);
        }
        else{
            lift1.setPower(0);
            lift2.setPower(0);
        }

        // armServo
        if (gamepad1.dpad_up) {
            armServo.setPosition(0.7);
        }
        else if (gamepad1.dpad_down){
            armServo.setPosition(0.1);
        }

        // Grabber servo (Need to test values!)
        if (gamepad2.dpad_left){
            // close
            grabber.setPosition(1.0);
        }
        else if (gamepad2.dpad_right){
            // open
            grabber.setPosition(0.0);
        }
        else{
            // stop
            grabber.setPosition(0.5);
        }


    }
}
