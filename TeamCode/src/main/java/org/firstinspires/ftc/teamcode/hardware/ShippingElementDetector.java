package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ShippingElementDetector {

    DistanceSensor sensorDistanceLeft;
    DistanceSensor sensorDistanceRight;
    public static final String NEITHER = "neither";
    public static final String LEFT = "left";
    public static final String RIGHT = "right";

    public ShippingElementDetector(DistanceSensor sensorDistanceLeft, DistanceSensor sensorDistanceRight){
        this.sensorDistanceLeft = sensorDistanceLeft;
        this.sensorDistanceRight = sensorDistanceRight;
    }

    public String detectPringle(){
        String results = NEITHER;
        double leftDistance = sensorDistanceLeft.getDistance(DistanceUnit.CM);
        double rightDistance = sensorDistanceRight.getDistance(DistanceUnit.CM);
        if (leftDistance < 10 && rightDistance < 10){
            if (leftDistance < rightDistance){
                results = LEFT;
            }else{
                results = RIGHT;
            }
        }else if(leftDistance<10){
            results = LEFT;
        }else if(rightDistance<10){
            results = RIGHT;
        }




        return results;
    }

}
