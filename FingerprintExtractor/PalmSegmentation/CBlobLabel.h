#ifndef CBlobLabel_H_
#define CBlobLabel_H_

class CBlobLabel
{
 public:
  CBlobLabel();
  ~CBlobLabel();
  
    void BlobAnalysisInt(unsigned char* ImageIN, int Width, int Height, int* ImageOut, int Th, int& AnzBlobs);
	void BlobAnalysisIntOpt(unsigned char* ImageIN, int Width, int Height, int* ImageOut, int Th, int& AnzBlobs);
};

#endif /*CBlobLabel_H_*/

