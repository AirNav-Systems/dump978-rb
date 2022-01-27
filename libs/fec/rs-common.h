/* Stuff common to all the general-purpose Reed-Solomon codecs
 * Copyright 2004 Phil Karn, KA9Q
 * May be used under the terms of the GNU Lesser General Public License (LGPL)
 */

#define RS_ERROR_DEG_LAMBDA_ZERO -1
#define RS_ERROR_IMPOSSIBLE_ERR_POS -2
#define RS_ERROR_DEG_LAMBDA_NEQ_COUNT -3
#define RS_ERROR_NOT_A_CODEWORD -4

/* Reed-Solomon codec control block */
struct rs {
  int mm;              /* Bits per symbol */
  int nn;              /* Symbols per block (= (1<<mm)-1) */
  data_t *alpha_to;     /* log lookup table */
  data_t *index_of;     /* Antilog lookup table */
  data_t *genpoly;      /* Generator polynomial */
  int nroots;     /* Number of generator roots = number of parity symbols */
  int fcr;        /* First consecutive root, index form */
  int prim;       /* Primitive element, index form */
  int iprim;      /* prim-th root of 1, index form */
  int pad;        /* Padding bytes in shortened block */
};

static inline int modnn(struct rs *rs,int x){
  while (x >= rs->nn) {
    x -= rs->nn;
    x = (x >> rs->mm) + (x & rs->nn);
  }
  return x;
}
