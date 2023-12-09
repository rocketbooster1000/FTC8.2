package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;

public class LiftRedZoneCommand extends CommandBase {
    LiftSubsystem m_subsystem;

    public LiftRedZoneCommand(LiftSubsystem subsystem){
        this.m_subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize(){
        m_subsystem.setTarget(Constants.Lift.RED_ZONE);
    }

    @Override
    public boolean isFinished(){
        return m_subsystem.getPosition() >= Constants.Lift.RED_ZONE;
    }

}
