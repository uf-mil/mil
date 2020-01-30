# TCP sample streaming protocol specification

The sonar board captures samples at 1.2 MHz (±0.5ppm) over 4 channels.

One sample is represented as 4 big-endian signed 16-bit integers, one per channel. The data stream that `./publish` provides over TCP is just this individual sample data unit, repeated at the sample rate. When a client connects to `./publish`'s TCP server, the first samples it receives will be from around the time when the connection is made.

Samples can also be saved to a file by using a command like `nc localhost 2000 > capture.raw`. This format specification also applies to that kind of file.

An example capture file from Aug 3rd 2019 at the TRANSDEC is available [here](http://sylphase.com/files/oof.bin). It can be listened to by running `./downsample ~/Downloads/oof.bin 0 10 out.wav 20` and then `aplay out.wav`.

The total data rate of a sample stream is `1.2 MHz * 4 channels * 2 bytes/cycle/channel = 9.6 MB/s`, which is somewhat hefty. Consumers of this data need to be written somewhat cleverly if they're written in a relatively slow language like Python. Don't try to loop over the samples; instead, convert the data to a NumPy array and just do NumPy operations on it.

## Example

Let's use this data series as an example:

| Sample # | Channel 0 | Channel 1 | Channel 2 | Channel 3 |
|---|---|---|---|---|
| 0 | 42 | 0 | 0 | 0 |
| 1 | 0 | 42 | 0 | 0 |
| 2 | 0 | 0 | 42 | 0 |
| 3 | 0 | 0 | 0 | 42 |
| 4 | 1 | 2 | 3 | 4 |
| 5 | 5 | 6 | 7 | 8 |
| 6 | -9 | -10 | -11 | -12 |
| 6 | 1013 | 1014 | 1015 | 1016 |
| 7 | -1017 | -1018 | -1019 | -1020 |

This is 8 samples, which corresponds to about 6.7 microseconds of captured sound.

This would be encoded into the following binary data (displayed here in hexadecimal form):

    002A0000000000000000002A0000000000000000002A0000000000000000002A00010002000300040005000600070008FFF7FFF6FFF5FFF403F503F603F703F8FC07FC06FC05FC04

Here's the same data with line breaks added to divide it up into samples and spaces added to separate the channels:

    002A 0000 0000 0000
    0000 002A 0000 0000
    0000 0000 002A 0000
    0000 0000 0000 002A
    0001 0002 0003 0004
    0005 0006 0007 0008
    FFF7 FFF6 FFF5 FFF4
    03F5 03F6 03F7 03F8
    FC07 FC06 FC05 FC04

The correspondence between this and the table is straightforward.
