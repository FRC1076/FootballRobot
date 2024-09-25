package frc.robot.utils;

import frc.robot.Constants.BallisticsConstants;

public class ballistics {
    //Basic ballistics computation
    /**
     * Calculates the distance that a football will be launched, ignoring air resistance
     * Uses the range equation from wikipedia
     * @param speed the speed of the football, in m/s
     * @return the distance that the football will be launched, in meters
     */
    public static double rangeVacuum(double speed){
        double shooterAngleRad = Math.toRadians(BallisticsConstants.kShooterAngle);
        return ((speed * Math.cos(shooterAngleRad))/BallisticsConstants.kGravity) * 
            ((speed * Math.sin(shooterAngleRad)) + 
            Math.sqrt(
                (Math.pow(speed,2) * Math.pow(Math.sin(shooterAngleRad),2)) + 
                (2 * BallisticsConstants.kGravity * BallisticsConstants.kShooterHeight)
            ));
    }
}
