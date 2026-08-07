class DriverConfig: #TODO: Add parameters related to driver
    def __init__(self, Kp, Ki, Kd, saturationValue, scalefacor):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = 0
        self.previousError = 0.0
        self.totalError=0.0
        self.dH = 100
        self.scalefactor = scalefactor
        self.saturationValue = saturationValue
        self.proportional = 0
        self.integral = 0
        self.derivative = 0
        self.output = 0

    def updateGainValues(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def setTotalError (self, error):
        self.totalError = error
    
    def setSaturationPoint(self, saturationValue):
        self.saturationValue = saturationValue

    def updateSetpoint(self,setpoint : float):
        if self.saturationValue > setpoint:
            self.setpoint = setpoint
        else:
            self.setpoint = self.saturationValue

        if self.saturationValue == 0:
            self.setpoint = setpoint
    
    def dtUpdate (self, new_dt):
        self.dt = new_dt

    def computeOutput (self, sensorValue : float):
        currentError = (self.setpoint - sensorValue)
        proportional = (self.Kp * currentError)/(self.scalefactor)
        integral = ((self.Ki * (self.totalError + currentError)) / self.dH / self.scalefactor)
        derivative = (self.Kd *  (currentError - self.previousError) * self.dH / self.scalefactor)
        self.proportional = proportional
        self.integral = integral
        self.derivative = derivative

        self.previousError = currentError
        self.totalError += currentError

        output = proportional + integral + derivative
        self.output = output

    def getKp(self):
        return self.Kp
    
    def getKi(self):
        return self.Ki

    def getKd(self):
        return self.Kd

    def getSetpoint(self):
        return self.setpoint
    
    def getPreviousError(self):
        return self.previousError

    def getTotalError(self):
        return self.totalError

    def getOutput(self):
        return self.output

    def getProportional(self):
        return self.proportional

    def getIntegral(self):
        return self.integral

    def getDerivative(self):
        return self.derivative

    def getSaturationValue(self):
        return self.saturationValue