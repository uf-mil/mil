from .thruster_comm import UnavailableThrusterException
from .thruster_comm import UndeclaredThrusterException
from .thruster_comm import VRCSRException
from .thruster_comm import Sub8SerialException
from .thruster_comm import ThrusterModel
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
