import wpilib
import phoenix5

class MyRobot(wpilib.TimedRobot):
    def shoot(self):
        self.topWheel.set(phoenix5.TalonSRXControlMode.PercentOutput,1)
        self.bottomWheel.set(phoenix5.TalonSRXControlMode.PercentOutput,-1)
    def reverseShoot(self):
        self.topWheel.set(phoenix5.TalonSRXControlMode.PercentOutput,-1)
        self.bottomWheel.set(phoenix5.TalonSRXControlMode.PercentOutput,1)
    

    def robotInit(self):
        self.leftMotor = phoenix5.TalonFX(13)
        self.rightMotor = phoenix5.TalonFX(14)
        self.topWheel = phoenix5.TalonSRX(7)
        self.bottomWheel = phoenix5.TalonSRX(12)
        self.controller = wpilib.XboxController(0)
    
    def teleopPeriodic(self):
        self.leftMotor.set(phoenix5.TalonFXControlMode.PercentOutput, self.controller.getLeftY())
        self.rightMotor.set(phoenix5.TalonFXControlMode.PercentOutput,self.controller.getRightY())
        if self.controller.getXButton() == True:
            self.shoot()
        elif self.controller.getBButton() == True:
            self.reverseShoot()
        else:
            self.topWheel.set(phoenix5.TalonSRXControlMode.PercentOutput, 0)
            self.bottomWheel.set(phoenix5.TalonSRXControlMode.PercentOutput, 0)
    
        

    