import numpy as np



x = np.uint16(0x0001)
y = x.byteswap()
print(y)
