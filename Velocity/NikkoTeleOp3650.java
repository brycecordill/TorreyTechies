package org.firstinspires.ftc.teamcode.Velocity;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Bryce on 11/2/2016.
 */
@TeleOp(name="Operation: Telly (NIKKO)", group="3650")
@Disabled
public class NikkoTeleOp3650 extends OpMode {

    //HI v2

    //assigning state variables
    DcMotor rDrive, lDrive, collector, shooter;
    Servo colorServo, ballServo;
    ColorSensor colorSensor;
    OpticalDistanceSensor ods;
    LightSensor light;




    @Override
    public void init() {

        // linking variables to hardware components
        lDrive = hardwareMap.dcMotor.get("lDrive");
        rDrive = hardwareMap.dcMotor.get("rDrive");
        collector = hardwareMap.dcMotor.get("collector");
        shooter = hardwareMap.dcMotor.get("shooter");
        colorServo = hardwareMap.servo.get("colorServo");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        ballServo = hardwareMap.servo.get("ballServo");
        ods = hardwareMap.opticalDistanceSensor.get("ods");
        light = hardwareMap.lightSensor.get("light");

        //Reversing direction of R Drive so it spins the correct way
        rDrive.setDirection(DcMotor.Direction.REVERSE);
        //shooter.setDirection(DcMotor.Direction.REVERSE);
        ballServo.setPosition(0.14);
        colorSensor.enableLed(false);

    }

    @Override
    public void loop() {

        //set motor controls to joystick


        shooter.setPower(gamepad2.right_trigger);






        if (gamepad1.right_trigger>0){
            lDrive.setPower(gamepad1.left_stick_y*.5);
            rDrive.setPower(gamepad1.right_stick_y*.5);

        }

        else if (gamepad1.right_trigger==0){

            lDrive.setPower(gamepad1.left_stick_y);
            rDrive.setPower(gamepad1.right_stick_y);


        }


        //servo controls for the movement of the color sensor
        if (gamepad1.left_bumper) {
            colorServo.setPosition(-1.00);
        }
        else if (gamepad1.right_bumper) {
            colorServo.setPosition(1.00);
        }

        //shooter flippy thing servo
        if (gamepad2.dpad_up){//up
            ballServo.setPosition(1.00);
        }
        else{//down
            ballServo.setPosition(0.14);
        }



        /*//moving ball collector
        if(gamepad1.y){
            collector.setPower(1.00);//up
        }
        if(gamepad1.a){
            collector.setPower(-1.00);//down
        }
        if (gamepad1.x){
            collector.setPower(0);//stopped
        }*/
        if (gamepad2.left_bumper){
            collector.setPower(-1.0);
        }
        else if(gamepad2.right_bumper){
            collector.setPower(1.00);
        }
        else {
            collector.setPower(0);
        }

        //shows values from color sensor on driver station phone
        telemetry.addData("Distance value",ods.getLightDetected());
        telemetry.addData("Light", light.getLightDetected());
        telemetry.addData("Red ", colorSensor.red());
        telemetry.addData("Green ", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
    }
}





















