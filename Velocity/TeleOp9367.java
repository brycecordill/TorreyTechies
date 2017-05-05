package org.firstinspires.ftc.teamcode.Velocity;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Bryce on 11/2/2016.
 * Modified by Wenlong on 12/5/2016.
 */
@TeleOp(name="TeleOp 01/07", group="9367")
public class TeleOp9367 extends OpMode {

    //assigning state variables
    DcMotor rDrive, lDrive, collector, shooter, elevator;
    Servo ballServo;
    //   ColorSensor colorSensor;

    long setTime;
    boolean trigger = true;


    @Override
    public void init() {

        // linking variables to hardware components
        lDrive = hardwareMap.dcMotor.get("lDrive");
        rDrive = hardwareMap.dcMotor.get("rDrive");
        collector = hardwareMap.dcMotor.get("collector");
        shooter = hardwareMap.dcMotor.get("shooter");
        elevator = hardwareMap.dcMotor.get("elevator");
        ballServo = hardwareMap.servo.get("ballServo");

    }

    @Override
    public void loop() {

        lDrive.setPower(-gamepad1.left_stick_y);
        rDrive.setPower(gamepad1.right_stick_y);
        //lDrive.setPower(-gamepad1.left_stick_y+0.5*gamepad1.left_stick_x);
        //rDrive.setPower(gamepad1.left_stick_y+0.5*gamepad1.left_stick_x);



        //set shooter
        if(gamepad1.a) {

            if (trigger) {
                setTime = System.currentTimeMillis();
                trigger = false;
            }

            else {
                if (System.currentTimeMillis() - setTime > 350 && System.currentTimeMillis() - setTime < 450) {
                    shooter.setPower(0);
                }
                else if (System.currentTimeMillis() - setTime > 450) {
                    shooter.setPower(1.0);
                }
                else {
                    shooter.setPower(-0.15);
                }
            }

        }


        else{
            shooter.setPower(0);
            trigger = true;
        }


        //set collector
        if (gamepad2.right_bumper){//change to gamepad2 for competition
            collector.setPower(-1.0);
        }
        else if(gamepad2.left_bumper){//change to gamepad2 for competition
            collector.setPower(1.00);
        }
        else {
            collector.setPower(0);
        }

        //set elevator
        if(gamepad2.right_trigger == 1){//change to gamepad2 for competition
            elevator.setPower(-1.00);
        }
        else if(gamepad2.left_trigger == 1){ //change to gamepad2 for competition
            elevator.setPower(1.00);
        }
        else{
            elevator.setPower(0);
        }


        if(gamepad2.x){//change to gamepad2 for competition
            ballServo.setPosition(0.15);
        }
        else{
            ballServo.setPosition(0.57);
        }

        //shows values from color sensor on driver station phone
        //  telemetry.addData("Red ", colorSensor.red());
        //  telemetry.addData("Green ", colorSensor.green());
        //  telemetry.addData("Blue ", colorSensor.blue());
    }
}
