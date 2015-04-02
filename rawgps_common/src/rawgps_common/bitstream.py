class BitStream(object):
    # reads MSB of each byte first
    
    def __init__(self, bytes):
        self.bytes = bytes
        self.bit_pos = 0
    
    def _get_bit(self, bit_pos):
        byte_pos = bit_pos // 8
        subbyte_pos = bit_pos % 8
        return 1 if ord(self.bytes[byte_pos]) & (1 << (7-subbyte_pos)) else 0
    
    def read(self, bits):
        res = 0
        for i in xrange(bits):
            res *= 2
            res += self._get_bit(self.bit_pos)
            self.bit_pos += 1
        return res
    
    def read_signed(self, bits):
        return (self.read(bits) + 2**bits/2) % 2**bits - 2**bits//2
    
    def at_end(self):
        return self.bit_pos == len(self.bytes)*8
