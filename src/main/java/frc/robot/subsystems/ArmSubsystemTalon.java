package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystemTalon extends SubsystemBase {

    private final TalonSRX armMotor;
    
    /**
     * This subsytem that controls the arm.
     */
    public ArmSubsystemTalon () {

    // Set up the arm motor as a brushed motor
    armMotor = new TalonSRX(Constants.ArmConstants.ARM_MOTOR_ID);

    
    }

    @Override
    public void periodic() {
    }
    /** 
     * This is a method that makes the arm move at your desired speed
     *  Positive values make it spin forward and negative values spin it in reverse
     * 
     * @param speed motor speed from -1.0 to 1, with 0 stopping it
     */
    public void runArm(double speed){
        armMotor.set(ControlMode.PercentOutput, speed);
        System.out.print("runArm:  "); System.out.println(speed);
    }
}