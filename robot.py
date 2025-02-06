import wpilib
import wpilib.drive
from wpilib import SmartDashboard,RobotBase
from magicbot import will_reset_to

from wpimath import applyDeadband
from rev import SparkMax,SparkMaxConfig


class GloruisCreation(wpilib.TimedRobot):
    SWAG_BARRIER = 0.1
    SWAG_MULTIPLIER = 10
    MAX_SWAG_LEVEL = 9000
    SWAG_PERIOD = 500

    

    def robotInit(self):
        BRUSHLESS = SparkMax.MotorType.kBrushless
        self.frontleftmotor = SparkMax(5,BRUSHLESS)
        self.frontrightmotor = SparkMax(50,BRUSHLESS)
        self.backleftmotor = SparkMax(51,BRUSHLESS)
        self.backrightmotor = SparkMax(52,BRUSHLESS)
        self.xbox = wpilib.XboxController(0)
        self.ps = wpilib.PS5Controller(1)

        self.rightmotor = wpilib.MotorControllerGroup(
            self.frontrightmotor, self.backrightmotor
        )
        self.leftmotor = wpilib.MotorControllerGroup(
            self.frontleftmotor, self.backleftmotor
        )
        self.rightmotor.setInverted(True)

        self.robotDrive = wpilib.drive.DifferentialDrive(
            self.leftmotor, self.rightmotor
        )
        self.oldMove = 0
        self.oldRotate = 0
        self.swagLevel = 0
        self.swagPeriod = 0
        self.multi = 0.0
    def swagDrive(self, moveValue, rotateValue):
        """Custom drive function that incorporates 'swag' logic
        https://www.chiefdelphi.com/t/introducing-swagdrive-the-drive-code-of-the-future/129519
        """
        if self.xbox.getAButton() or self.ps.getCrossButton():
            swagmode = True
            moveToSend = moveValue
            rotateToSend = rotateValue

            if self.swagPeriod == 0:
                moveDiff = abs(moveValue) + abs(self.oldMove)
                rotateDiff = abs(rotateValue) + abs(self.oldRotate)

                if moveDiff < self.SWAG_BARRIER:
                    moveToSend = (moveDiff * self.SWAG_MULTIPLIER) + moveValue
                else:
                    self.swagLevel += 90

                if rotateDiff < self.SWAG_BARRIER:
                    rotateToSend = (rotateDiff * self.SWAG_MULTIPLIER) + rotateValue
                else:
                    self.swagLevel += 90

                if self.swagLevel > self.MAX_SWAG_LEVEL:
                    self.swagPeriod = self.SWAG_PERIOD
                    self.swagLevel = 0

                SmartDashboard.putNumber("Swag Level", self.swagLevel)
                SmartDashboard.putNumber("Move Diff", moveDiff)
            else:
                moveToSend = 0
                rotateToSend = 1.0
                self.swagPeriod -= 1

            # Call arcadeDrive with modified move and rotate values
            
            self.robotDrive.arcadeDrive(moveToSend, rotateToSend)

            self.oldMove = moveValue
            self.oldRotate = rotateValue
                
        else:
            self.robotDrive.arcadeDrive(moveValue, rotateValue)




    def teleopPeriodic(self):
        """Called periodically during operator control"""
        if self.xbox.getLeftBumper() and self.xbox.getRightBumper():
            self.multi = 0.8
        else:
            self.multi = 0.5
        if applyDeadband(self.xbox.getLeftY(), 0.15) == 0 and applyDeadband(self.xbox.getRightX(), 0.15) == 0:
            self.leftStick = self.ps.getLeftY()
            self.rightStick = self.ps.getRightX()
        elif applyDeadband(self.ps.getLeftY(), 0.15) == 0 and applyDeadband(self.ps.getRightX(), 0.15) == 0:
            self.leftStick = self.xbox.getLeftY()
            self.rightStick = self.xbox.getRightX()
        else:
            self.leftStick = (self.xbox.getLeftY() + self.ps.getLeftY()) / 2.0
            self.rightStick = (self.xbox.getRightX() + self.ps.getRightX()) / 2.0
            
        moveValue = applyDeadband(-self.leftStick, 0.15)
        rotateValue = applyDeadband(-self.rightStick, 0.15)
        SmartDashboard.putNumber("left", self.leftStick)
        SmartDashboard.putNumber("right", self.rightStick)
        self.swagDrive(moveValue * self.multi, rotateValue * self.multi)

