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
 * Created by Bryce on 12/1/2016.
 */

@TeleOp(name="Telly Single", group="3650")
@Disabled
public class OpTellySingle3650 extends OpMode {

    //TeleOp using a single controller
    //for more comments, go to OperationTelly3650

    //assigning state variables
    DcMotor rDrive, lDrive, collector, shooter;
    ColorSensor colorSensor;
    Servo forePush, aftPush;
    OpticalDistanceSensor ods;
    LightSensor light;
    double aftNeutral, foreNeutral;




    @Override
    public void init() {
        aftNeutral = .2; //1.0 is max
        foreNeutral = 1.0; //0 is max

        // linking variables to hardware components
        lDrive = hardwareMap.dcMotor.get("lDrive");
        rDrive = hardwareMap.dcMotor.get("rDrive");
        collector = hardwareMap.dcMotor.get("collector");
        shooter = hardwareMap.dcMotor.get("shooter");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");


        ods = hardwareMap.opticalDistanceSensor.get("ods");
        light = hardwareMap.lightSensor.get("light");

        //button pushing servos
        forePush = hardwareMap.servo.get("forePush");
        aftPush = hardwareMap.servo.get("aftPush");

        //Reversing direction of R Drive so it spins the correct way
        rDrive.setDirection(DcMotor.Direction.REVERSE);
        rDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        colorSensor.enableLed(false);

    }

    @Override
    public void loop() {

        //set motor controls to joystick
        lDrive.setPower(gamepad1.left_stick_y*.7);
        rDrive.setPower(gamepad1.right_stick_y*.7);

        shooter.setPower(gamepad1.right_trigger);

        if (gamepad1.left_bumper){
            collector.setPower(1.0);
        }
        else if(gamepad1.right_bumper){
            collector.setPower(-1.00);
        }
        else {
            collector.setPower(0);
        }


        if(gamepad1.b){
            forePush.setPosition(0);
        }
        else if(gamepad1.x){
            aftPush.setPosition(1.00);
        }
        else{
            aftPush.setPosition(aftNeutral);
            forePush.setPosition(foreNeutral);
        }

        //shows values from color sensor on driver station phone
        telemetry.addData("Distance value",ods.getLightDetected());
        telemetry.addData("Light", light.getLightDetected());
        telemetry.addData("Red ", colorSensor.red());
        telemetry.addData("Green ", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
    }
}

