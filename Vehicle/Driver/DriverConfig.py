from Functions import PIDcontroller

class DriverConfig: #TODO: Add parameters related to driver
    def __init__(self):
        self.pid = PIDcontroller(10, 10, 0, 80, 10)
        self.pedalPosition = 0
        self.targetSpeed = 0
        self.thresholdDiscrepancy = 5
        self.initializationThreshold = 0
        self.clampingMethod = 0

    # how to accurately tune PID values? how to determine the saturation value?
    # where is target speed being read from?
    # where is current speed being read from?
    # how to associate the pedal position with speed?

    def updatePIDController(self, pidSetpoint, sensorValue):
        currentError = pidSetpoint - sensorValue
        match self.clampingMethod:
            case 0:
                self.pid.antiWindupFlag = False
                return
            case 4:
                if currentError < 0:
                    self.pid.antiWindupFlag = True
                    self.pid.totalError -= getPreviousError(self.pid)
                else:
                    self.pid.antiWindupFlag = False
                return
            case 6:
                if self.pid.proportional + self.pid.integral + self.pid.derivative + sensorValue > self.pid.saturationValue:
                    self.pid.totalError -= getPreviousError(self.pid)
                    pid.antiWindupFlag = True
                else:
                    self.pid.antiWindupFlag = False
                return
            
        
        updateSetpoint(self.pid, pidSetpoint)
        computeOutput(self.pid, sensorValue)