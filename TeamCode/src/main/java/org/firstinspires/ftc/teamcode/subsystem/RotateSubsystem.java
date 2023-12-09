package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.Constants;

public class RotateSubsystem extends SubsystemBase {

    public enum BinaryState{
        BASE_STATE,
        EXCITED_STATE
    }


    private Servo rotationServo;

    private BinaryState state;



    public RotateSubsystem(HardwareMap hardwareMap){
        rotationServo = hardwareMap.get(Servo.class, "Rotation_Servo");
        rotationServo.setDirection(Servo.Direction.REVERSE);
        rotationServo.setPosition(Constants.PassThrough.SLIDE_SERVO_ZERO_POSITION);
        state = BinaryState.BASE_STATE;

    }

    public void rotate(){
        switch(state){
            case BASE_STATE:
                rotationServo.setPosition(Constants.PassThrough.SLIDE_SERVO_ROTATED_POSITION);
                state = BinaryState.EXCITED_STATE;
                break;
            case EXCITED_STATE:
                rotationServo.setPosition(Constants.PassThrough.SLIDE_SERVO_ZERO_POSITION);
                state = BinaryState.BASE_STATE;
                break;
        }
    }


}
