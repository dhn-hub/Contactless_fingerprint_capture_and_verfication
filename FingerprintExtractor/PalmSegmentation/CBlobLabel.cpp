// =============================================================================
// INCLUDES
// =============================================================================

#include <iostream>
#include <math.h>
#include <stdlib.h>
//#include <QStringList>
#include "CBlobLabel.h"
#include <string.h>

//#include <qsize.h>
//#include <qpoint.h>

CBlobLabel::CBlobLabel()
{
}

CBlobLabel::~CBlobLabel()
{
}

void CBlobLabel::BlobAnalysisInt(unsigned char* ImageIN, int Width, int Height, int* ImageOut, int Th, int& AnzBlobs)
{
	int *m_Label=new int[Height*Width];
	memset(m_Label, 0, Height*Width*sizeof(int));

	unsigned char* ImageINPtr=ImageIN;
	int* m_LabelPtr=m_Label;
	for(int i=0;i<Height*Width;i++) 
	{
		if(*ImageINPtr>Th) 
			*m_LabelPtr=1;
		ImageINPtr++;
		m_LabelPtr++;
	}

	int tempOffset=(Height-1)*Width;
	for(int x=0;x<Width;x++)
	{	
		m_Label[x]=0;
		m_Label[tempOffset+x]=0;
	}
	
	tempOffset=Width-1;
	for(int y=0;y<Height;y++)	
	{	
		int tempOffestX=(y)*Width;
		m_Label[tempOffestX]=0;
		m_Label[tempOffestX+tempOffset]=0;
	}

	//m_Label grob setzen 
	int LabelIndex=1;
	for(int y=1;y<Height-1;y++) 
	{
		int tempOffsetY1=(y-1)*Width;
		int tempOffsetY2=(y)*Width;
		for(int x=1;x<Width-1;x++) 
		{
			if(m_Label[y*Width+x]!=0)
			{
				int Lab=0;
				if(m_Label[tempOffsetY1+x-1]>Lab) 	Lab=m_Label[tempOffsetY1+x-1];
				if(m_Label[tempOffsetY1+x-0]>Lab) 	Lab=m_Label[tempOffsetY1+x-0];
				if(m_Label[tempOffsetY1+x+1]>Lab)		Lab=m_Label[tempOffsetY1+x+1];
				if(m_Label[tempOffsetY2+x-1]>Lab)		Lab=m_Label[tempOffsetY2+x-1];
				if (Lab==0)
				{
					LabelIndex++;
					Lab=LabelIndex-1;
				}
				m_Label[tempOffsetY2+x]=Lab;
			}
		}
	}

	//Graph aufbauen
	// Laufe durch das Bild, und sobal zwei Labels sich berühren, setze ein Kante vom größeren zum kleinsten Label
	int* mySortArray=new int[LabelIndex];
	for(int i=0;i<LabelIndex;i++) 	mySortArray[i]=i; // Selbstzeiger der Knoten

	for(int y=Height-2;y>0;y--) 
		for(int x=Width-2;x>0;x--) 
	{
		if(m_Label[y*Width+x]!=0)
		{
			int neu,alt;
			neu=alt=mySortArray[m_Label[y*Width+x]]; // Aktueller Index

			if(m_Label[(y-1)*Width+x-1]!=0 && mySortArray[m_Label[(y-1)*Width+x-1]]!=neu) 	
					neu=mySortArray[m_Label[(y-1)*Width+x-1]];
			else if(m_Label[(y-1)*Width+x+0]!=0 && mySortArray[m_Label[(y-1)*Width+x+0]]!=neu)		
					neu=mySortArray[m_Label[(y-1)*Width+x+0]];
			else if(m_Label[(y-1)*Width+x+1]!=0 && mySortArray[m_Label[(y-1)*Width+x+1]]!=neu) 	
					neu=mySortArray[m_Label[(y-1)*Width+x+1]];
			else if(m_Label[(y+0)*Width+x-1]!=0 && mySortArray[m_Label[(y+0)*Width+x-1]]!=neu)	
					neu=mySortArray[m_Label[(y+0)*Width+x-1]];
			else if(m_Label[(y+0)*Width+x+1]!=0 && mySortArray[m_Label[(y+0)*Width+x+1]]!=neu)		
					neu=mySortArray[m_Label[(y+0)*Width+x+1]];
			else if(m_Label[(y+1)*Width+x-1]!=0 && mySortArray[m_Label[(y+1)*Width+x-1]]!=neu)
					neu=mySortArray[m_Label[(y+1)*Width+x-1]];
			else if(m_Label[(y+1)*Width+x+0]!=0 && mySortArray[m_Label[(y+1)*Width+x+0]]!=neu)	
					neu=mySortArray[m_Label[(y+1)*Width+x+0]];
			else if(m_Label[(y+1)*Width+x+1]!=0 && mySortArray[m_Label[(y+1)*Width+x+1]]!=neu)	 	
					neu=mySortArray[m_Label[(y+1)*Width+x+1]];
			if(neu!=alt)	
			{
				for(int i=1;i<LabelIndex;i++)
				{
					if (mySortArray[i]==alt) 
					{
						mySortArray[i]=neu;
					}
				}
			}
		}
	}

	int* SortArray=new int[LabelIndex];
	memcpy(SortArray, mySortArray, LabelIndex*sizeof(int));

	//Wurzel suchen
	int* SortArray2=new int[LabelIndex];
	SortArray2[0]=SortArray[0];	// Background
	for(int i=1;i<LabelIndex;i++) 
	{
		int NextIdx=i;
		/*while(SortArray[NextIdx]!=NextIdx) // Solange kein Selbstzeiger...
		{
			NextIdx=SortArray[NextIdx];
		}*/
		bool RootFound=false;
		int RootIndex=0;
		for(int j=1;j<LabelIndex;j++) // Maximale Anzal an Suchanfragen
		{
			if(NextIdx==SortArray[NextIdx])
			{
				RootFound=true;
				RootIndex=NextIdx;
				break;
			}
			NextIdx=SortArray[NextIdx];
		}
		if(RootFound)
		{
			SortArray2[i]=RootIndex;
		}
	}

	// Zusammenfassen und durchnummerieren
	int m_AnzLabels=1;
	int* SortArrayFlags=new int[LabelIndex];
	memset(SortArrayFlags, 0, LabelIndex*sizeof(int));
	for(int i=1;i<LabelIndex;i++)
	{
		if(SortArray2[i])
		{
			if(SortArrayFlags[i]==0)
			{
				int IndexToSearch=SortArray2[i];
				
				for(int j=1;j<LabelIndex;j++)
				{
					if(SortArray2[j]==IndexToSearch)
					{
						SortArray2[j]=m_AnzLabels;
						SortArrayFlags[j]=1;
					}
					
				}
				m_AnzLabels++;
			}
		}
	}

	for(int i=0;i<LabelIndex;i++)
	{
		SortArray[i]=SortArray2[SortArray[i]];
	}
	delete [] SortArrayFlags;
	
	memset(ImageOut, 0, Width*Height*sizeof(int));

	int* ImageOutPtr=ImageOut;
	m_LabelPtr=m_Label;
	for(int i=0;i<Height*Width;i++) 
	{
		if(*m_LabelPtr>0) 
			*ImageOutPtr=SortArray[*m_LabelPtr];

		m_LabelPtr++;
		ImageOutPtr++;
	}

	/*bool testPrint=false;
	for(int xi=1; xi<Width-1;  xi++) 
	for(int yi=1; yi<Height-1; yi++) 
	{
		if(ImageOut[xi+yi*Width]!=0)
		{
			if((ImageOut[xi+yi*Width]!=ImageOut[(xi-1)+(yi-1)*Width]	&& ImageOut[(xi-1)+(yi-1)*Width]!=0 ) ||
				(ImageOut[xi+yi*Width]!=ImageOut[(xi)+(yi-1)*Width]	&& ImageOut[(xi)+(yi-1)*Width]!=0 ) ||
				(ImageOut[xi+yi*Width]!=ImageOut[(xi+1)+(yi-1)*Width] && ImageOut[(xi+1)+(yi-1)*Width]!=0 ) ||
				(ImageOut[xi+yi*Width]!=ImageOut[(xi-1)+(yi)*Width]	&& ImageOut[(xi-1)+(yi)*Width]!=0 ) ||
				(ImageOut[xi+yi*Width]!=ImageOut[(xi+1)+(yi)*Width]	&& ImageOut[(xi+1)+(yi)*Width]!=0 ) ||
				(ImageOut[xi+yi*Width]!=ImageOut[(xi-1)+(yi+1)*Width] && ImageOut[(xi-1)+(yi+1)*Width]!=0 ) ||
				(ImageOut[xi+yi*Width]!=ImageOut[(xi)+(yi+1)*Width]	&& ImageOut[(xi)+(yi+1)*Width]!=0 ) ||
				(ImageOut[xi+yi*Width]!=ImageOut[(xi+1)+(yi+1)*Width]	&& ImageOut[(xi+1)+(yi+1)*Width]!=0 ) )
			{
				int t=0;
				std::cerr << " Labeling Error " << std::endl;
				testPrint=true;
			}			
		}	
	}*/

	// Testausgabe der Blobs / Erzeugung des Blob-Bildes / Ermittlung der Blobgrößen
	// m_AnzLabels-1 -> Anzahl der der Blobs
	AnzBlobs=m_AnzLabels-1;

	delete [] mySortArray;
	delete [] m_Label;
	delete [] SortArray; 
	delete [] SortArray2; 
}


void CBlobLabel::BlobAnalysisIntOpt(unsigned char* ImageIN, int Width, int Height, int* ImageOut, int Th, int& AnzBlobs)
{
	int ImageSize=Height*Width;
	int *m_Label=new int[Height*Width];
	memset(m_Label, 0, Height*Width*sizeof(int));

	unsigned char* ImageINPtr=ImageIN;
	int* m_LabelPtr=m_Label;
	for(int i=0;i<ImageSize;i++) 
	{
		if(*ImageINPtr>Th) 
			*m_LabelPtr=1;
		ImageINPtr++;
		m_LabelPtr++;
	}

	int tempOffset=(Height-1)*Width;
	for(int x=0;x<Width;x++)
	{	
		m_Label[x]=0;
		m_Label[tempOffset+x]=0;
	}
	
	tempOffset=Width-1;
	for(int y=0;y<Height;y++)	
	{	
		int tempOffestX=(y)*Width;
		m_Label[tempOffestX]=0;
		m_Label[tempOffestX+tempOffset]=0;
	}

	//m_Label grob setzen 
	int LabelIndex=1;
	int* LabelPtr=m_Label;
	int* LabelPtrPrevLine=m_Label-Width;
	for(int y=0;y<Height;y++) 
	{
		for(int x=0;x<Width;x++) 
		{
			if(y==0 || x==0 || y==Height-1 || x==Width-1)
			{
				LabelPtr++;
				LabelPtrPrevLine++;
				continue;
			}
			if(*LabelPtr!=0)
			{
				int Lab=0;
				if(*(LabelPtrPrevLine-1)>Lab) 	Lab=*(LabelPtrPrevLine-1);
				if(*LabelPtrPrevLine>Lab) 		Lab=*LabelPtrPrevLine;
				if(*(LabelPtrPrevLine+1)>Lab)	Lab=*(LabelPtrPrevLine+1);
				if(*(LabelPtr-1)>Lab)	Lab=*(LabelPtr-1);
				if (Lab==0)
				{
					LabelIndex++;
					Lab=LabelIndex-1;
				}
				*LabelPtr=Lab;
			}
			LabelPtr++;
			LabelPtrPrevLine++;
		}
	}

	//Graph aufbauen
	// Laufe durch das Bild, und sobal zwei Labels sich berühren, setze ein Kante vom größeren zum kleinsten Label
	int* mySortArray=new int[LabelIndex];
	for(int i=0;i<LabelIndex;i++) 	mySortArray[i]=i; // Selbstzeiger der Knoten

	LabelPtr=m_Label+Width*Height-1;
	LabelPtrPrevLine=m_Label+Width*Height-Width-1;
	int* LabelPtrNextLine=m_Label+Width*Height+Width-1;

	for(int y=Height-1;y>=0;y--) 
		for(int x=Width-1;x>=0;x--) 
	{
		if(y==0 || x==0 || y==Height-1 || x==Width-1)
		{
			LabelPtr--;
			LabelPtrPrevLine--;
			LabelPtrNextLine--;
			continue;
		}
		if(*LabelPtr!=0)
		{
			int neu,alt;
			neu=alt=mySortArray[*LabelPtr];

			if(*(LabelPtrPrevLine-1)!=0 && neu!=mySortArray[*(LabelPtrPrevLine-1)]) 	
					neu=mySortArray[*(LabelPtrPrevLine-1)];
			else if(*(LabelPtrPrevLine)!=0 && neu!=mySortArray[*(LabelPtrPrevLine)])	
					neu=mySortArray[*(LabelPtrPrevLine)];
			else if(*(LabelPtrPrevLine+1)!=0 && neu!=mySortArray[*(LabelPtrPrevLine+1)]) 	
					neu=mySortArray[*(LabelPtrPrevLine+1)];
			else if(*(LabelPtr-1)!=0 && neu!=mySortArray[*(LabelPtr-1)])	
					neu=mySortArray[*(LabelPtr-1)];
			else if(*(LabelPtr+1)!=0 && neu!=mySortArray[*(LabelPtr+1)])	
					neu=mySortArray[*(LabelPtr+1)];
			else if(*(LabelPtrNextLine-1)!=0 && neu!=mySortArray[*(LabelPtrNextLine-1)])
					neu=mySortArray[*(LabelPtrNextLine-1)];
			else if(*(LabelPtrNextLine)!=0 && neu!=mySortArray[*(LabelPtrNextLine)])	
					neu=mySortArray[*(LabelPtrNextLine)];
			else if(*(LabelPtrNextLine+1)!=0 && neu!=mySortArray[*(LabelPtrNextLine+1)])
					neu=mySortArray[*(LabelPtrNextLine+1)];
			if(neu!=alt)	
			{
				for(int i=1;i<LabelIndex;i++)
					if (mySortArray[i]==alt) 
					{
						mySortArray[i]=neu;
					}
			}
		}
		LabelPtr--;
		LabelPtrPrevLine--;
		LabelPtrNextLine--;
	}

	int* SortArray=new int[LabelIndex];
	memcpy(SortArray, mySortArray, LabelIndex*sizeof(int));

	//Wurzel suchen
	int* SortArray2=new int[LabelIndex];
	SortArray2[0]=SortArray[0];	// Background
	for(int i=1;i<LabelIndex;i++) 
	{
		int NextIdx=i;
		/*while(SortArray[NextIdx]!=NextIdx) // Solange kein Selbstzeiger...
		{
			NextIdx=SortArray[NextIdx];
		}*/
		bool RootFound=false;
		int RootIndex=0;
		for(int j=1;j<LabelIndex;j++) // Maximale Anzal an Suchanfragen
		{
			if(NextIdx==SortArray[NextIdx])
			{
				RootFound=true;
				RootIndex=NextIdx;
				break;
			}
			NextIdx=SortArray[NextIdx];
		}
		if(RootFound)
		{
			SortArray2[i]=RootIndex;
		}
	}

	// Zusammenfassen und durchnummerieren
	int m_AnzLabels=1;
	int* SortArrayFlags=new int[LabelIndex];
	memset(SortArrayFlags, 0, LabelIndex*sizeof(int));
	for(int i=1;i<LabelIndex;i++)
	{
		if(SortArray2[i])
		{
			if(SortArrayFlags[i]==0)
			{
				int IndexToSearch=SortArray2[i];
				
				for(int j=1;j<LabelIndex;j++)
				{
					if(SortArray2[j]==IndexToSearch)
					{
						SortArray2[j]=m_AnzLabels;
						SortArrayFlags[j]=1;
					}
					
				}
				m_AnzLabels++;
			}
		}
	}

	for(int i=0;i<LabelIndex;i++)
	{
		SortArray[i]=SortArray2[SortArray[i]];
	}
	delete [] SortArrayFlags;
	
	memset(ImageOut, 0, Width*Height*sizeof(int));

	int* ImageOutPtr=ImageOut;
	m_LabelPtr=m_Label;
	for(int i=0;i<Height*Width;i++) 
	{
		if(*m_LabelPtr>0) 
			*ImageOutPtr=SortArray[*m_LabelPtr];

		m_LabelPtr++;
		ImageOutPtr++;
	}

	/*bool testPrint=false;
	for(int xi=1; xi<Width-1;  xi++) 
	for(int yi=1; yi<Height-1; yi++) 
	{
		if(ImageOut[xi+yi*Width]!=0)
		{
			if((ImageOut[xi+yi*Width]!=ImageOut[(xi-1)+(yi-1)*Width]	&& ImageOut[(xi-1)+(yi-1)*Width]!=0 ) ||
				(ImageOut[xi+yi*Width]!=ImageOut[(xi)+(yi-1)*Width]	&& ImageOut[(xi)+(yi-1)*Width]!=0 ) ||
				(ImageOut[xi+yi*Width]!=ImageOut[(xi+1)+(yi-1)*Width] && ImageOut[(xi+1)+(yi-1)*Width]!=0 ) ||
				(ImageOut[xi+yi*Width]!=ImageOut[(xi-1)+(yi)*Width]	&& ImageOut[(xi-1)+(yi)*Width]!=0 ) ||
				(ImageOut[xi+yi*Width]!=ImageOut[(xi+1)+(yi)*Width]	&& ImageOut[(xi+1)+(yi)*Width]!=0 ) ||
				(ImageOut[xi+yi*Width]!=ImageOut[(xi-1)+(yi+1)*Width] && ImageOut[(xi-1)+(yi+1)*Width]!=0 ) ||
				(ImageOut[xi+yi*Width]!=ImageOut[(xi)+(yi+1)*Width]	&& ImageOut[(xi)+(yi+1)*Width]!=0 ) ||
				(ImageOut[xi+yi*Width]!=ImageOut[(xi+1)+(yi+1)*Width]	&& ImageOut[(xi+1)+(yi+1)*Width]!=0 ) )
			{
				int t=0;
				std::cerr << " Labeling Error " << std::endl;
				testPrint=true;
			}			
		}	
	}*/

	/*unsigned char* LabelImageTest=new unsigned char[Width*Height];
	
	if(testPrint)
	{
		for(int i=0; i<Width*Height; i++)
			LabelImageTest[i]=(unsigned char)m_Label[i];
	
		SaveDebugImage(LabelImageTest1, Width, Height, 1, "BlobImage_Eval", 1);
		SaveDebugImage(LabelImageTest, Width, Height, 1, "BlobImage_Eval", 2);
	}
	
	delete [] LabelImageTest1;
	delete [] LabelImageTest;*/

	// Testausgabe der Blobs / Erzeugung des Blob-Bildes / Ermittlung der Blobgrößen
	// m_AnzLabels-1 -> Anzahl der der Blobs
	AnzBlobs=m_AnzLabels-1;

	delete [] mySortArray;
	delete [] m_Label;
	delete [] SortArray; 
	delete [] SortArray2; 
}

