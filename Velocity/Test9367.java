package org.firstinspires.ftc.teamcode.Velocity;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.reflect.Array;
import java.util.ArrayList;


@TeleOp(name="Test9367", group="9367")
public class Test9367 extends OpMode {
    ColorSensor colorSensor;
    OpticalDistanceSensor distanceSensor1, distanceSensor2;
    LightSensor lightSensor;

    double lightSensorValue = 0, lightDetected1 = 0, lightDetected2 = 0, rawLightDetected1 = 0, rawLightDetected2 = 0; //, rawValue1 = 0, rawValue2 = 0;
    long setTime = 0;
    boolean trigger = true;
    double count = 0;

    int epoch = 1, totalEpoch = 20;
    long epochTime = 2000;


    @Override
    public void init() {

        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        lightSensor = hardwareMap.lightSensor.get("lightSensor");
        distanceSensor1 = hardwareMap.opticalDistanceSensor.get("distanceSensor1");
        distanceSensor2 = hardwareMap.opticalDistanceSensor.get("distanceSensor2");

    }

    @Override
    public void loop() {

       // colorSensor.enableLed(false);

        telemetry.addData("currentRed ", colorSensor.red());
        telemetry.addData("currentGreen ", colorSensor.green());
        telemetry.addData("currentBlue ", colorSensor.blue());
        telemetry.addData("currentlight ", lightSensor.getLightDetected());


        telemetry.addData("currentdistance1 ", distanceSensor1.getLightDetected());
        telemetry.addData("currentdistance2 ", distanceSensor2.getLightDetected());
        telemetry.addData("currentdistance1rawlight ", distanceSensor1.getRawLightDetected());
        telemetry.addData("currentdistance2rawlight ", distanceSensor2.getRawLightDetected());
        telemetry.addData("currentdistance1noMethod ", distanceSensor1);
        telemetry.addData("currentdistance2noMethod ", distanceSensor2);



        while(epoch <= totalEpoch){


        if (trigger) {
            setTime = System.currentTimeMillis();
            trigger = false;
        }


        if(System.currentTimeMillis() - setTime < epochTime){

            lightDetected1 += distanceSensor1.getLightDetected();
            lightDetected2 += distanceSensor2.getLightDetected();
            rawLightDetected1 += distanceSensor1.getRawLightDetected();
            rawLightDetected2 += distanceSensor2.getRawLightDetected();
            lightSensorValue += lightSensor.getLightDetected();
            count++;

        }


        else{

            telemetry.addData("Ep "+epoch+" - distance1 ", lightDetected1/count);
            telemetry.addData("Ep "+epoch+" - distance2 ", lightDetected2/count);
            telemetry.addData("Ep "+epoch+" - distance1rawLight ", rawLightDetected1/count);
            telemetry.addData("Ep "+epoch+" - distance1rawLight ", rawLightDetected2/count);
            telemetry.addData("Ep "+epoch+" - lightSensorValue ", lightSensorValue/count);



            lightDetected1 = 0;
            lightDetected2 = 0;
            rawLightDetected1 = 0;
            rawLightDetected2 = 0;
            lightSensorValue = 0;

            count = 0;
            trigger = true;
            //epoch++;

            }

            epoch++;
        }


    }








}
