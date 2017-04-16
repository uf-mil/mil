from .thruster_comm import ThrusterPort
from .thruster_fake import FakeThrusterPort


def thruster_comm_factory(port_info, thruster_definitions, fake=False):
    '''
    Return the appropriate thruster communication class
    Purpose:
      - Use the same code to run both simulated thrusters and real thrusters
    '''
    if fake:
        return FakeThrusterPort(port_info, thruster_definitions)
    else:
        return ThrusterPort(port_info, thruster_definitions)

