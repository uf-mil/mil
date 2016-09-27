import serial
import math
class Sabertooth2x12:
  def _map(self,x, in_min, in_max, out_min, out_max):
    return int(math.ceil( (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min ) )
  def _contrain(self,x,low,high):
    return max(min(x,high),low)
  def setMotor1(speed):
    contrained_num = self._contrain(speed,-1.0,1.0)
    send_char = self._contrain(self._map(contrained_num,-1.0,1.0,1,127))
    self.ser.write(send_char)
  def setMotor2(speed):
    contrained_num = self._contrain(speed,-1.0,1.0)
    send_char = int(self._map(contrained_num,-1.0,1.0,128,255))
    self.ser.write(send_char)
  #def test(self):
    #print self._contrain(-22,-1.0,1.0)
    #print self._map(0,-1.0,1.0,1,127)
    #print self._map(0,-1.0,1.0,128,255)
    #print self._map(1,-1.0,1.0,1,127)
    #print self._map(1,-1.0,1.0,128,255)
    #print self._map(-1,-1.0,1.0,1,127)
    #print self._map(-1,-1.0,1.0,128,255)
  def __init__(self,filename,):
    #self.test()
    self.filename = filename
    self.ser = serial.Serial(filename)
