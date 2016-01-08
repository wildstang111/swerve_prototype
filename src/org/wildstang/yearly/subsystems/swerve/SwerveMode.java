package org.wildstang.yearly.subsystems.swerve;

public interface SwerveMode
{

   public SwerveBaseState calculateNewState(SwerveBaseState p_prevState, double... args);
}
