import wpilib

MOTOR_TOP_PORT = None
MOTOR_BOTTOM_PORT = None

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        print("Initalizing")
        self.timer = wpilib.Timer()
        self.input = wpilib.Joystick(0)

        self.speed = 0.0              # The % output that the motors should go at
        self.last_speed = 0.0         # Only update the % output when it changes

        self.motor_top = wpilib.Spark(MOTOR_TOP_PORT)
        self.motor_bottom = wpilib.Spark(MOTOR_BOTTOM_PORT)

    def teleopInit(self):
        print("Entering teleop")

    def teleopPeriodic(self):
        # Update speed of motors
        if (self.speed != self.last_speed):
            self.motor_top.set(self.speed)
            self.motor_bottom.set(-self.speed)
            self.last_speed = self.speed
        
        # Reset to 0 speed
        if (self.input.getRawButtonPressed(1)):
            self.speed = 0

        # "Emergency" Reset to 0 speed
        if (self.input.getRawButtonPressed(2)):
            self.motor_top.set(0)
            self.motor_bottom.set(0)
            self.speed = 0
            self.last_speed = 0
        
        # Update speed based on joystick values
        if (self.input.getRawAxis(1) > 0.8 and self.speed < 1):
            self.speed += 0.01
            print(self.speed)
        elif (self.input.getRawAxis(1) < -0.8 and self.speed > -1):
            self.speed -= 0.01
            print(self.speed)
        

    def teleopExit(self):
        print("Exited teleop")