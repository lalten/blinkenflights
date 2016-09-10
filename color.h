/*
 * color.h
 *
 *  Created on: Sep 10, 2016
 *      Author: laurenz
 */

#ifndef COLOR_H_
#define COLOR_H_

void HSVtoRGB( float *r, float *g, float *b, float h, float s, float v ) {

	while(h>=360)h-=360;
  int i;
  float f, p, q, t;

  if(s == 0) {
    *r = *g = *b = v;
    return;
  }

  h /= 60;
  i = floor(h);
  f = h - i;
  p = v * (1 - s);
  q = v * (1 - s * f);
  t = v * (1 - s * (1 - f));

  switch(i) {
    case 0:
      *r = v;
      *g = t;
      *b = p;
      break;
    case 1:
      *r = q;
      *g = v;
      *b = p;
      break;
    case 2:
      *r = p;
      *g = v;
      *b = t;
      break;
    case 3:
      *r = p;
      *g = q;
      *b = v;
      break;
    case 4:
      *r = t;
      *g = p;
      *b = v;
      break;
    default:
      *r = v;
      *g = p;
      *b = q;
      break;
  }
}



#endif /* COLOR_H_ */
