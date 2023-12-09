package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;

public class LiftOverrideMoveCommand extends CommandBase {
    LiftSubsystem lift;


    public LiftOverrideMoveCommand(LiftSubsystem subsystem){
        this.lift = subsystem;




        addRequirements(subsystem);
    }

    @Override
    public void execute(){
        lift.override(Constants.Lift.MOTOR_SLIDE_RESET_POWER);
    }
}
