/* LibTomCrypt, modular cryptographic library -- Tom St Denis
 *
 * LibTomCrypt is a library that provides various cryptographic
 * algorithms in a highly modular and flexible manner.
 *
 * The library is free for all purposes without any express
 * guarantee it works.
 *
 * Tom St Denis, tomstdenis@gmail.com, http://libtom.org
 */

#ifndef _NTZVLIYKNNMYVSDR_
#define _NTZVLIYKNNMYVSDR_

#include <cassert>
#include <cstdint>
#include <cstring>

namespace uf_subbus_protocol {


struct sha256_state {
    uint64_t length;
    uint32_t state[8], curlen;
    unsigned char buf[64];
};

void LOAD32H(uint32_t &x, uint8_t const *y) {
  x = (static_cast<uint32_t>(y[0] & 255)<<24) |
      (static_cast<uint32_t>(y[1] & 255)<<16) |
      (static_cast<uint32_t>(y[2] & 255)<< 8) |
      (static_cast<uint32_t>(y[3] & 255)<< 0);
}
void STORE32H(uint32_t const x, uint8_t *y) {
  y[0] = static_cast<uint8_t>((x>>24)&255);
  y[1] = static_cast<uint8_t>((x>>16)&255);
  y[2] = static_cast<uint8_t>((x>> 8)&255);
  y[3] = static_cast<uint8_t>((x>> 0)&255);
}
void STORE64H(uint64_t const x, uint8_t *y) {
  y[0] = static_cast<uint8_t>((x>>56)&255);
  y[1] = static_cast<uint8_t>((x>>48)&255);
  y[2] = static_cast<uint8_t>((x>>40)&255);
  y[3] = static_cast<uint8_t>((x>>32)&255);
  y[4] = static_cast<uint8_t>((x>>24)&255);
  y[5] = static_cast<uint8_t>((x>>16)&255);
  y[6] = static_cast<uint8_t>((x>> 8)&255);
  y[7] = static_cast<uint8_t>((x>> 0)&255);
}
uint32_t RORc(uint32_t x, uint32_t y) {
  return (((static_cast<uint32_t>(x)&0xFFFFFFFFUL)>>static_cast<uint32_t>((y)&31)) |
    (static_cast<uint32_t>(x)<<static_cast<uint32_t>(32-((y)&31)))) & 0xFFFFFFFFUL;
}
template<typename T>
T const & MIN(T const &x, T const &y) {
  return x < y ? x : y;
}


/**
  @file sha256.c
  LTC_SHA256 by Tom St Denis
*/

/* the K array */
static const uint32_t K[64] = {
    0x428a2f98UL, 0x71374491UL, 0xb5c0fbcfUL, 0xe9b5dba5UL, 0x3956c25bUL,
    0x59f111f1UL, 0x923f82a4UL, 0xab1c5ed5UL, 0xd807aa98UL, 0x12835b01UL,
    0x243185beUL, 0x550c7dc3UL, 0x72be5d74UL, 0x80deb1feUL, 0x9bdc06a7UL,
    0xc19bf174UL, 0xe49b69c1UL, 0xefbe4786UL, 0x0fc19dc6UL, 0x240ca1ccUL,
    0x2de92c6fUL, 0x4a7484aaUL, 0x5cb0a9dcUL, 0x76f988daUL, 0x983e5152UL,
    0xa831c66dUL, 0xb00327c8UL, 0xbf597fc7UL, 0xc6e00bf3UL, 0xd5a79147UL,
    0x06ca6351UL, 0x14292967UL, 0x27b70a85UL, 0x2e1b2138UL, 0x4d2c6dfcUL,
    0x53380d13UL, 0x650a7354UL, 0x766a0abbUL, 0x81c2c92eUL, 0x92722c85UL,
    0xa2bfe8a1UL, 0xa81a664bUL, 0xc24b8b70UL, 0xc76c51a3UL, 0xd192e819UL,
    0xd6990624UL, 0xf40e3585UL, 0x106aa070UL, 0x19a4c116UL, 0x1e376c08UL,
    0x2748774cUL, 0x34b0bcb5UL, 0x391c0cb3UL, 0x4ed8aa4aUL, 0x5b9cca4fUL,
    0x682e6ff3UL, 0x748f82eeUL, 0x78a5636fUL, 0x84c87814UL, 0x8cc70208UL,
    0x90befffaUL, 0xa4506cebUL, 0xbef9a3f7UL, 0xc67178f2UL
};

/* Various logical functions */
uint32_t Ch(uint32_t x, uint32_t y, uint32_t z) { return z ^ (x & (y ^ z)); }
uint32_t Maj(uint32_t x, uint32_t y, uint32_t z) { return ((x | y) & z) | (x & y); }
uint32_t S(uint32_t x, uint32_t n) { return RORc(x, n); }
uint32_t R(uint32_t x, uint32_t n) { return (x & 0xFFFFFFFFUL) >> n; }
uint32_t Sigma0(uint32_t x) { return S(x, 2) ^ S(x, 13) ^ S(x, 22); }
uint32_t Sigma1(uint32_t x) { return S(x, 6) ^ S(x, 11) ^ S(x, 25); }
uint32_t Gamma0(uint32_t x) { return S(x, 7) ^ S(x, 18) ^ R(x, 3); }
uint32_t Gamma1(uint32_t x) { return S(x, 17) ^ S(x, 19) ^ R(x, 10); }

/* compress 512-bits */
static void  sha256_compress(sha256_state &md, uint8_t const *buf)
{
    uint32_t s[8], W[64], t0, t1;
    uint32_t t;
    int i;

    /* copy state into S */
    for (i = 0; i < 8; i++) {
        s[i] = md.state[i];
    }

    /* copy the state into 512-bits into W[0..15] */
    for (i = 0; i < 16; i++) {
        LOAD32H(W[i], buf + (4*i));
    }

    /* fill W[16..63] */
    for (i = 16; i < 64; i++) {
        W[i] = Gamma1(W[i - 2]) + W[i - 7] + Gamma0(W[i - 15]) + W[i - 16];
    }

    /* Compress */
     for (i = 0; i < 64; ++i) {
         t0 = s[7] + Sigma1(s[4]) + Ch(s[4], s[5], s[6]) + K[i] + W[i];
         t1 = Sigma0(s[0]) + Maj(s[0], s[1], s[2]);
         s[3] += t0;
         s[7] = t0 + t1;
         
         t = s[7]; s[7] = s[6]; s[6] = s[5]; s[5] = s[4];
         s[4] = s[3]; s[3] = s[2]; s[2] = s[1]; s[1] = s[0]; s[0] = t;
     }

    /* feedback */
    for (i = 0; i < 8; i++) {
        md.state[i] = md.state[i] + s[i];
    }
}

/**
   Initialize the hash state
   @param md   The hash state you wish to initialize
   @return CRYPT_OK if successful
*/
void sha256_init(sha256_state &md)
{
    md.curlen = 0;
    md.length = 0;
    md.state[0] = 0x6A09E667UL;
    md.state[1] = 0xBB67AE85UL;
    md.state[2] = 0x3C6EF372UL;
    md.state[3] = 0xA54FF53AUL;
    md.state[4] = 0x510E527FUL;
    md.state[5] = 0x9B05688CUL;
    md.state[6] = 0x1F83D9ABUL;
    md.state[7] = 0x5BE0CD19UL;
}

/**
   Process a block of memory though the hash
   @param md     The hash state
   @param in     The data to hash
   @param inlen  The length of the data (octets)
   @return CRYPT_OK if successful
*/
void sha256_process(sha256_state &md, const uint8_t *in, uint32_t inlen)
{
    unsigned int block_size = 64;
    uint32_t n;
    assert(in);
    assert(md.curlen <= sizeof(md.buf));
    while (inlen > 0) {
        if (md.curlen == 0 && inlen >= block_size) {
           sha256_compress(md, in);
           md.length += block_size * 8;
           in             += block_size;
           inlen          -= block_size;
        } else {
           n = MIN(inlen, (block_size - md.curlen));
           memcpy(md.buf + md.curlen, in, static_cast<size_t>(n));
           md.curlen += n;
           in             += n;
           inlen          -= n;
           if (md.curlen == block_size) {
              sha256_compress(md, md.buf);
              md.length += 8*block_size;
              md.curlen = 0;
           }
       }
    }
}

/**
   Terminate the hash to get the digest
   @param md  The hash state
   @param out [out] The destination of the hash (32 bytes)
   @return CRYPT_OK if successful
*/
void sha256_done(sha256_state &md, uint8_t *out)
{
    int i;

    assert(out);
    assert(md.curlen < sizeof(md.buf));


    /* increase the length of the message */
    md.length += md.curlen * 8;

    /* append the '1' bit */
    md.buf[md.curlen++] = static_cast<uint8_t>(0x80);

    /* if the length is currently above 56 bytes we append zeros
     * then compress.  Then we can fall back to padding zeros and length
     * encoding like normal.
     */
    if (md.curlen > 56) {
        while (md.curlen < 64) {
            md.buf[md.curlen++] = static_cast<uint8_t>(0);
        }
        sha256_compress(md, md.buf);
        md.curlen = 0;
    }

    /* pad upto 56 bytes of zeroes */
    while (md.curlen < 56) {
        md.buf[md.curlen++] = static_cast<uint8_t>(0);
    }

    /* store length */
    STORE64H(md.length, md.buf+56);
    sha256_compress(md, md.buf);

    /* copy output */
    for (i = 0; i < 8; i++) {
        STORE32H(md.state[i], out+(4*i));
    }
}


}

#endif
