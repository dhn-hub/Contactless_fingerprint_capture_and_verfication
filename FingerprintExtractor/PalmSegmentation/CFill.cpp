// =============================================================================
// INCLUDES
// =============================================================================

#include <iostream>
#include <math.h>
#include <stdlib.h>
//#include <QStringList>
#include <list>
#include "CFill.h"
#include <string.h>

//#include <qsize.h>
//#include <qpoint.h>

#define M_PI 3.141528

typedef struct _XSegment
{
	short x1;
	short y1;
	short x2;
	short y2;
} XSegment;

  //Define declarations.

#define GreyMatch(color,target,m_delta) \
  ((((int) color-m_delta) <= (int) target) && \
    ((int) target <= ((int) color+m_delta)))
#define MaxStacksize  (1 << 15)
#define Push(up,left,right,m_delta) \
  if ((p < (segment_stack+MaxStacksize)) && (((up)+(m_delta)) >= 0) && \
      (((up)+(m_delta)) < (int)image->rows)) \
    { \
      p->y1=(up); \
      p->x1=(left); \
      p->x2=(right); \
      p->y2=(m_delta); \
      p++; \
    }
#define Push2(up,left,right,m_delta) \
  if ((p < (segment_stack+MaxStacksize)) && (((up)+(m_delta)) >= 0) && \
      (((up)+(m_delta)) < (int)height)) \
    { \
      p->y1=(up); \
      p->x1=(left); \
      p->x2=(right); \
      p->y2=(m_delta); \
      p++; \
    }

typedef struct _IMGImage
{
  unsigned int
    columns,
    rows;

  unsigned char
  *pixels;
} IMGImage;

/*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/

Cfill::Cfill()
{
  ;// (noch) nichts zu tun
}

Cfill::~Cfill()
{
	;// (noch) nichts zu tun
}

void Cfill::fill ( unsigned char *greyvalues,
		   unsigned int width,
		   unsigned int height,
		   int x,
		   int y,
		   unsigned char xcolor,
		   int m_delta)
{

	IMGImage fiximage;
	IMGImage *image;
	int offset, skip;
	int start;
	int x1, x2;

	unsigned char *pixel;

	XSegment *p;

	unsigned char color, target;

	XSegment
		*segment_stack;
 
	image = &fiximage;
	image->pixels = greyvalues;
	image->rows = height;
	image->columns = width;

  //  Check boundary conditions.

  if ((y < 0) || (y >= (int)image->rows))
    return;
  if ((x < 0) || (x >= (int)image->columns))
    return;
  target=image->pixels[y*image->columns+x];
  color = xcolor;
  if (GreyMatch(color,target,m_delta))
    return;

   // Allocate segment stack.

  segment_stack=(XSegment *) malloc(MaxStacksize*sizeof(XSegment));
  if (segment_stack == (XSegment *) NULL)
    {
     // Warning("Unable to recolor image","Memory allocation failed");
      return;
    }

   // Push initial segment on stack.

  start=0;
  p=segment_stack;
  Push(y,x,x,1);
  Push(y+1,x,x,-1);
  while (p > segment_stack)
  {
    
    //  Pop segment off stack.
    p--;
    x1=p->x1;
    x2=p->x2;
    offset=p->y2;
    y=p->y1+offset;
 
      //Recolor neighboring pixels.
    for (x=x1; x >= 0 ; x--)
    {
      pixel=image->pixels+(y*image->columns+x);
      if (!GreyMatch(*pixel,target,m_delta))
        break;
      *pixel=color;
    }
    skip=x >= x1;
    if (!skip)
      {
        start=x+1;
        if (start < x1)
          Push(y,start,x1-1,-offset);
        x=x1+1;
      }
    do
    {
      if (!skip)
        {
          for ( ; x < (int)image->columns; x++)
          {
            pixel=image->pixels+(y*image->columns+x);
            if (!GreyMatch(*pixel,target,m_delta))
              break;
            *pixel=color;
          }
          Push(y,start,x-1,offset);
          if (x > (x2+1))
            Push(y,x2+1,x-1,-offset);
        }
      skip=false;
      for (x++; x <= x2 ; x++)
      {
        pixel=image->pixels+(y*image->columns+x);
        if (GreyMatch(*pixel,target,m_delta))
          break;
      }
      start=x;
    } while (x <= x2);
  }
  free((char *) segment_stack);
}

//////////////////////////////////////////7 ADDITIONAL COLOR NORMALIZATION //////////////////////////////

static void RGB709toXYZ(double r, double g, double b, double& x, double& y, double& z) {
	r/=255.0;
	g/=255.0;
	b/=255.0;

  float mat[3][3] =
  {{ 0.412453, 0.35758 , 0.180423},
   { 0.212671, 0.71516 , 0.072169},
   { 0.019334, 0.119193, 0.950227}};
  
  x = mat[0][0] * r + mat[0][1] * g + mat[0][2] * b;
  y = mat[1][0] * r + mat[1][1] * g + mat[1][2] * b;
  z = mat[2][0] * r + mat[2][1] * g + mat[2][2] * b;
}

static void XYZtoRGB709(double r, double g, double b, double& x, double& y, double& z) {
  float mat[3][3] =
  {{  3.2402542, -1.5371385 , -0.4985314},
   { -0.9692660, 1.8760108 ,   0.0415560},
   {  0.0556434, -0.2040259,   1.0572252}};
  
  x = mat[0][0] * r + mat[0][1] * g + mat[0][2] * b;
  y = mat[1][0] * r + mat[1][1] * g + mat[1][2] * b;
  z = mat[2][0] * r + mat[2][1] * g + mat[2][2] * b;
  	x*=255.0;
	y*=255.0;
	z*=255.0;
}

static void convert_rgb_2_cieluv(double R, double G, double B, double& l, double& u, double& v) {
  double xn=0.9505;
  double yn=1.0;
  double zn=1.0888;

	if(R==0 && G==0 && B==0)
	{
		l=0;
		u=0;
		v=0;
		return;
	}

  double x=0.0;
  double y=0.0;
  double z=0.0;
  //RGB709toXYZ(255, 255, 255, xn, yn, zn);
  RGB709toXYZ(R, G, B, x, y, z);

  if((y / yn) > (216.0/24389.0) )
	l = -16 + 116.0 * pow(y / yn, 1./3);
  else
	l = (24389.0/27.0) * (y / yn);

	double ul=4*x /(x+15*y+3*z);
	double unl=4*xn /(xn+15*yn+3*zn);
	double vl=9*y /(x+15*y+3*z);
	double vnl=9*yn /(xn+15*yn+3*zn);

	 u=13*l*(ul-unl);
	 v=13*l*(vl-vnl);
}

static void convert_cieluv_2_rgb(double l, double u, double v, double& R, double &G, double &B) {
  double xn=0.9505;
  double yn=1.0;
  double zn=1.0888;

	/*if(R==0 && G==0 && B==0)
	{
		l=0;
		u=0;
		v=0;
		return;
	}*/

  // Luv2xyz
  double x=0.0;
  double y=0.0;
  double z=0.0;

  double u0=(4*xn)/(xn+15*yn+3*zn);
  double v0=(9*yn)/(xn+15*yn+3*zn);

  if(l>216.0/27.0)
	y=((l+16)/116)*((l+16)/116)*((l+16)/116); // pow 3
  else
	y=l/(24389.0/27.0); // pow 3

  double a=1.0/3.0*((52*l)/(u+13*l*u0)-1);
  double b=-5*y;
  double c=-1.0/3.0;
  double d=y*((39*l)/(v+13*l*v0)-5);

  x=(d-b)/(a-c);
  
  z=x*a+b;

  // xyz2rgb
  double r=0.0;
  double g=0.0;
  double bl=0.0;
  XYZtoRGB709(x, y, z, r, g, bl);

  R=r;
  G=g;
  B=bl;
}


typedef struct _XPixel
{
	short x;
	short y;
} XPixel;

double calcColorDistance(unsigned char *image, int width, int height, XPixel RefCoord, XPixel TestPixel)
{
	if(RefCoord.x<0 || RefCoord.x>=width || RefCoord.y<0 || RefCoord.y>=height ||
	   TestPixel.x<0 || TestPixel.x>=width || TestPixel.y<0 || TestPixel.y>=height)
	   return -1;

	double R_ref=(double)image[RefCoord.x+width*RefCoord.y];
	double G_ref=(double)image[RefCoord.x+width*RefCoord.y+width*height];
	double B_ref=(double)image[RefCoord.x+width*RefCoord.y+width*height*2];
	double R_test=(double)image[TestPixel.x+width*TestPixel.y];
	double G_test=(double)image[TestPixel.x+width*TestPixel.y+width*height];
	double B_test=(double)image[TestPixel.x+width*TestPixel.y+width*height*2];

	double l_ref, u_ref, v_ref;
	double l_test, u_test, v_test;
	convert_rgb_2_cieluv(R_ref, G_ref, B_ref, l_ref, u_ref, v_ref);
	convert_rgb_2_cieluv(R_test, G_test, B_test, l_test, v_test, v_test);

	return sqrt((u_ref-u_test)*(u_ref-u_test)+(v_ref-v_test)*(v_ref-v_test));
	double angle_ref=(atan2(u_ref, v_ref)+M_PI)*360;
	double angle_test=(atan2(u_test, v_test)+M_PI)*360;

	double diffangle=(angle_ref-angle_test);

	if(diffangle<-180)
		diffangle+=180;
	if(diffangle>180)
		diffangle-=180;

	if(diffangle<0)
		diffangle*=-1;
	
	return diffangle;
	//return sqrt((u_ref-u_test)*(u_ref-u_test)+(v_ref-v_test)*(v_ref-v_test));
}

double calcColorDistance(unsigned char *image, int width, int height, double MeanL, double MeanU, double MeanV, XPixel TestPixel)
{
	if(TestPixel.x<0 || TestPixel.x>=width || TestPixel.y<0 || TestPixel.y>=height)
	   return -1;

	double R_test=(double)image[TestPixel.x+width*TestPixel.y];
	double G_test=(double)image[TestPixel.x+width*TestPixel.y+width*height];
	double B_test=(double)image[TestPixel.x+width*TestPixel.y+width*height*2];

	double l_test, u_test, v_test;
	convert_rgb_2_cieluv(R_test, G_test, B_test, l_test, u_test, v_test);

	if(MeanL<0)
		return sqrt((MeanU-u_test)*(MeanU-u_test)+(MeanV-v_test)*(MeanV-v_test));
	else
		return sqrt((MeanL-l_test)*(MeanL-l_test)+(MeanU-u_test)*(MeanU-u_test)+(MeanV-v_test)*(MeanV-v_test));
}

void Cfill::fillColor ( unsigned char *image,
		   unsigned int width,
		   unsigned int height,
		   unsigned int planes,
		   unsigned char* result_mask,
		   int x,
		   int y,
		   double& meanL,
		   double& meanU,
		   double& meanV,
		   double m_delta, bool Init)
{

	unsigned char* work_mask=new unsigned char[width*height];
	memset(work_mask, 0, width*height);

    std::list<XPixel> pixelStack;
	
	if(x>=0 && x<width && y>=0 && y<height)
	{
		XPixel newItem;
		newItem.x=x;newItem.y=y;

		pixelStack.push_back(newItem);
		work_mask[newItem.x+newItem.y*width]=1;

		double MeanL=meanL;
		double MeanU=meanU;
		double MeanV=meanV;
		if(!Init)
		{
			double R_ref=(double)image[newItem.x+width*newItem.y];
			double G_ref=(double)image[newItem.x+width*newItem.y+width*height];
			double B_ref=(double)image[newItem.x+width*newItem.y+width*height*2];

			double l_ref, u_ref, v_ref;
			convert_rgb_2_cieluv(R_ref, G_ref, B_ref, l_ref, u_ref, v_ref);
			MeanL=l_ref;
			MeanU=u_ref;
			MeanV=v_ref;
		}

		while(pixelStack.size()!=0)
		{
			XPixel pixelInWork=pixelStack.front();
			
			// Check Neighbourhood
			XPixel N1; N1.x=pixelInWork.x-1; N1.y=pixelInWork.y;
			XPixel N2; N2.x=pixelInWork.x;   N2.y=pixelInWork.y-1;
			XPixel N3; N3.x=pixelInWork.x;   N3.y=pixelInWork.y+1;
			XPixel N4; N4.x=pixelInWork.x+1; N4.y=pixelInWork.y;
			
			double Dist1=calcColorDistance(image, width, height, MeanL, MeanU, MeanV, N1);
			double Dist2=calcColorDistance(image, width, height, MeanL, MeanU, MeanV, N2);
			double Dist3=calcColorDistance(image, width, height, MeanL, MeanU, MeanV, N3);
			double Dist4=calcColorDistance(image, width, height, MeanL, MeanU, MeanV, N4);

			if(Dist1>=0 && Dist1<m_delta && work_mask[N1.x+N1.y*width]==0)
			{
				pixelStack.push_back(N1);
				work_mask[N1.x+N1.y*width]=1;
			}
			if(Dist2>=0 && Dist2<m_delta && work_mask[N2.x+N2.y*width]==0)
			{
				pixelStack.push_back(N2);
				work_mask[N2.x+N2.y*width]=1;
			}
			if(Dist3>=0 && Dist3<m_delta && work_mask[N3.x+N3.y*width]==0)
			{
				pixelStack.push_back(N3);
				work_mask[N3.x+N3.y*width]=1;
			}
			if(Dist4>=0 && Dist4<m_delta && work_mask[N4.x+N4.y*width]==0)
			{
				pixelStack.push_back(N4);
				work_mask[N4.x+N4.y*width]=1;
			}

			pixelStack.pop_front();
		}
		
		meanL=MeanL;
		meanU=MeanU;
		meanV=MeanV;
	}

	memcpy(result_mask, work_mask, width*height);
	delete [] work_mask;
}

