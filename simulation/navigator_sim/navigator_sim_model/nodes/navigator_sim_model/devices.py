from __future__ import division

from twisted.protocols.basic import LineReceiver

class ThrusterProtocol(LineReceiver):
    delimiter = '\r'
    def __init__(self, thruster_id, thrusters):
        self.thruster_id = thruster_id
        self.thrusters = thrusters
    
    def connectionMade(self):
        LineReceiver.connectionMade(self)
        print 'thruster', self.thruster_id, 'connection made'
    
    def lineReceived(self, line):
        line = line.strip()
        if line == "":
            return
        if line.startswith('1'):
            self.setSpeed(int(line[1:]))
        elif line.startswith('2'):
            self.setSpeed(0)
        elif line.startswith('3'):
            self.setSpeed(-int(line[1:]))
        else:
            print 'unknown line from thruster: %r' % line
        self.transport.write(line[0] + '\r')
    
    def setSpeed(self, speed):
        self.thrusters[self.thruster_id] = speed/200
    
    def connectionLost(self, reason):
        print 'thruster', self.thruster_id, 'connection lost', reason
    
    def doStop(self): pass
