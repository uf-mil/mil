import logging
import serial
import time

MAGICWORD = 0x41504954;
XMT_START_WORD = 0x58535441;
RCV_START_WORD = 0x52535454;

OP_BOOTTABLE = 0x58535907;
OP_SEQREADEN = 0x58535963;
OP_SECTIONLOAD = 0x58535901;
OP_CSECTIONLOAD = 0x58535909;
OP_SECTIONFILL = 0x5853590A;
OP_FXNEXEC = 0x5853590D;
OP_JUMP = 0x58535905;
OP_JUMPCLOSE = 0x58535906;
OP_CRCEN = 0x58535903;
OP_CRCDIS = 0x58535904;
OP_CRCREQ = 0x58535902;
OP_READWAIT = 0x58535914;
OP_STARTOVER = 0x58535908;
OP_PINGDEVICE = 0x5853590B;

def op2ack(op):
    return (op & ~0x0F000000) | 0x02000000

class Exception(RuntimeError):
    pass

def delay():
    time.sleep(5/1000)

def word2str(word):
    return chr(word & 0xFF) + \
           chr((word >> 8) & 0xFF) + \
           chr((word >> 16) & 0xFF) + \
           chr((word >> 24) & 0xFF)

def str2word(s):
    vals = map(ord, s)
    return vals[0] + (vals[1] << 8) + (vals[2] << 16) + (vals[3] << 24)
    
def read_word_timeout(ser):
    s = ser.read(4)
    if len(s) < 4:
        raise Exception('Timeout while reading word')
    return str2word(s)
    
def sws(ser):
    """Start word sync"""
    logging.debug('Performing SWS')
    ser.write(chr(XMT_START_WORD >> 24))
    while True:
        response = ser.read(1)
        if len(response) == 0:
            raise Exception('Timeout during start word sync')
        if ord(response[0]) == (RCV_START_WORD >> 24):
            return True

def pos(ser, N=10):
    """Ping opcode sync"""
    logging.debug('Performing POS')
    ser.write(word2str(OP_PINGDEVICE))
    if read_word_timeout(ser) != op2ack(OP_PINGDEVICE):
        raise Exception('Invalid response to ping opcode')
    delay()

    ser.write(word2str(N))
    if read_word_timeout(ser) != N:
        raise Exception('Invalid response to ping count')

    for i in xrange(1, N+1):
        ser.write(word2str(i))
        if read_word_timeout(ser) != i:
            raise Exception('Invalid response to ping %d' % i)
    return True

def os(ser, op):
    ser.write(word2str(op))
    response = read_word_timeout(ser)
    if response != op2ack(op):
        raise Exception('Invalid response to opcode (%x, expected %x)' % (response, op2ack(op)))

def boot(ser, file):
    magic = str2word(file.read(4))
    if magic != MAGICWORD:
        raise Exception('Invalid magic word in file')
    
    sws(ser)
    pos(ser)

    while True:
        delay()
        op = str2word(file.read(4))
        os(ser, op)
        
        if op == OP_SECTIONLOAD or op == OP_CSECTIONLOAD:
            addr = str2word(file.read(4))
            ser.write(word2str(addr))
            size = str2word(file.read(4))
            ser.write(word2str(size))
            logging.debug('SECTIONLOAD of %d bytes to 0x%x' % (size, addr))
            ser.write(file.read(size))
        elif op == OP_FXNEXEC:
            args = str2word(file.read(4))
            ser.write(word2str(args))
            words = (args >> 16)*4
            logging.debug('FXNEXEC of %d bytes' % words)
            ser.write(file.read(words))
        elif op == OP_JUMPCLOSE:
            addr = str2word(file.read(4))
            logging.debug('JUMPLOAD to 0x%x' % addr)
            ser.write(word2str(addr))
            break
        elif op == OP_CRCEN:
            logging.debug('Enabled CRC')
        elif op == OP_CRCDIS:
            logging.debug('Disabled CRC')
        elif op == OP_CRCREQ:
            calc_crc = read_word_timeout(ser)
            real_crc = str2word(file.read(4))
            seek = str2word(file.read(4))
            if seek > 0x7FFFFFFF:
                seek = (seek-1) - 0xFFFFFFFF
            if real_crc == calc_crc:
                logging.debug('CRC passed')
            else:
                logging.debug('CRC failed')
                file.seek(seek, 1)
        else:
            raise Exception('Unknown opcode 0x%x' % op)

    donestr = ser.read(8)
    if donestr != "   DONE\0":
        raise Exception('Invalid done string: %s' % donestr)

    return True

def main():
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    with open('SimpleHyd2013.bin') as file:
        boot(ser, file)

if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    main()
    
