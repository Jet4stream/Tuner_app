#include <stdlib.h>
#include <math.h>
#include <fftw3.h>
#define N 3521 // currently that is the highest frequenccy we want (Nyquist)

int main(void) {
  fftw_complex in[N], out[N]; /* double [2] */
  fftw_plan p, q;
  int i;

  /* prepare a cosine wave */
  for (i = 0; i < N; i++) {
    in[i][0] = cos(3 * 2*M_PI*i/N); // changed to the analog input from the mic
  }

  /* forward Fourier transform, save the result in 'out' */
  p = fftw_plan_dft_1d(N, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
  fftw_execute(p);
  for (i = 0; i < N; i++)
    printf("freq: %3d %+9.5f %+9.5f I\n", i, out[i][0], out[i][1]);
  fftw_destroy_plan(p);

  fftw_cleanup();
  return 0;
}