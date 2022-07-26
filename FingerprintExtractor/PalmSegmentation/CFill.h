#ifndef CFILL_H_
#define CFILL_H_

//#ifndef _CFILL_H_
//#define _CFILL_H_
#include <iostream>
//! Flood-Fill in einem Bild
class Cfill
{
 public:
  Cfill();
  ~Cfill();
  

    void fill ( unsigned char *greyvalues,
	      unsigned int width,
	      unsigned int height,
	      int x,
	      int y,
	      unsigned char xcolor,
	      int m_delta
	      );

	void fillColor ( unsigned char *image,
		   unsigned int width,
		   unsigned int height,
		   unsigned int planes,
		   unsigned char* result_mask,
		   int x,
		   int y,
		   double& meanL,
		   double& meanU,
		   double& meanV,
		   double m_delta, bool Init);
};

#endif /*CFILL_H_*/

