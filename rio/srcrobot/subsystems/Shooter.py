from commands2.subsystem import Subsystem
from phoenix6 import ControlMode, TalonFX   
from constants import Constants
# VEX 775 30:1 gear ratio

class Shooter(Subsystem):
    def __init__(self):
        self.upMotor = TalonFX(Constants.Shooter.upCANID)
        self.downMotor = TalonFX(Constants.Shooter.downCANID)
        #self.upMotor.set(ControlMode.PercentOutput, joyShoot) #joystick value
        #self.downMotor.set(ControlMode.PercentOutput, joyShoot) #joystick value
        # define velocity(?)
        TalonFXConfigs = TalonFXConfigs()
        feedback_sensor_source: phoenix6.signals.spn_enums.FeedbackSensorSourceValue
        self.upMotor.configSelectedFeedbackSensor(
            FeedbackDevice.IntegratedSensor, 0, 0
        )
        pass

    def upShoot(self, Velocity):
        return self.run(self.upMotor.set(ControlMode.Velocity, Constants.Shooter.targetRPM)) #joystick value, 
    
    def downShoot(self, Velocity):
        return self.run(self.downMotor.set(ControlMode.Velocity, Constants.Shooter.targetRPM)) #joystick value

    def shoot(self):
        self.upShoot(Velocity) # how do i do this properly?
        self.downShoot(Velocity)    


    