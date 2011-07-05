#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include "ImageData.h"
#ifdef _DEBUG
#ifdef WIN32
#include <crtdbg.h>
#endif
#endif

/**
 *@brief コンストラクタ
 *@note イメージデータ用ポインタ、および画像の幅・高さをクリアする
 */
ImageData::ImageData(void) : 
	m_width(0), 
	m_height(0),
	m_image(0),
	m_cvImage(0),
	m_cvSimImage(0)

{
}

/**
 *@brief デストラクタ
 *@note 確保されているイメージデータ領域を破棄する
 */
ImageData::~ImageData(void)
{
	DeleteImage();
}

/**
 *@brief コピーコンストラクタ
 *@param src コピー元ImageDataインスタンス
 */
ImageData::ImageData(const ImageData& src)
{
	m_image			= 0;
	m_cvImage		= 0;
	m_cvSimImage	= 0;

	CreateImage(src.m_width ,src.m_height);
	memcpy(m_image, src.m_image, src.m_width * src.m_height);
}

/**
 *@brief イメージの畳み込みを行う。
 *@note 現画像を1/4サイズに畳み込んで新たなImageDataを作る。
 *@return 1/4サイズに畳み込んだ画像データ。
 */
ImageData ImageData::PyrDown()
{
	ImageData		result;
	unsigned long	width, height;

	unsigned long h_cnt, w_cnt;
	unsigned long x_pos, y_pos;

	width	= m_width / 2;
	height	= m_height / 2;
	result.CreateImage(width, height);

	for ( h_cnt = 0; h_cnt < height; h_cnt++ ) {	/* dstの高さループ */
		for( w_cnt = 0; w_cnt < width; w_cnt++ ) {	/* dstの幅ループ */
			x_pos = (w_cnt * 2);			/* 0,2,4,6,・・・srcの奇数列ピクセルを間引く */
			y_pos = (h_cnt * 2) * m_width;	/* 0,2,4,6,・・・srcの奇数行ピクセルを間引く */
			/*
				画像の右端からデータ格納・・・
				ソースのコピー対象ピクセルと右・下・右下(間引かれるピクセル)のピクセル値の平均を入れる
				・・・左端から右端に格納するので鏡像になる
			*/
			/*
				ターゲット座標	= X
				巻き込み対象	= O

				+ + + + + + +
				+ + + + + + +
				+ + + +X+O+ +
				+ + + +O+O+ +
				+ + + + + + +
				+ + + + + + +
			*/
			result[ (int)(h_cnt * width + (width - w_cnt - 1)) ]
				= (unsigned int)( m_image[ y_pos + x_pos ] +	
								  m_image[ y_pos + x_pos + m_width] +
								  m_image[ y_pos + x_pos + 1 ] +
								  m_image[ y_pos + x_pos + 1 + m_width ] ) / 4;
		}
	}

	return result;
}

/**
 *@brief イメージデータ形式変換
 *@param orgData CameraEyeCompから送られてきた画像データ列
 *@attention CameraEyeCompから送られてきたデータしか変換できない。
 */
void ImageData::SetData(TimedOctetSeq orgData)
{
	unsigned char	*imgData;

	DeleteImage();

	/* バイナリ配列を適切な状態に変更する */
	if(orgData.data.length() > 0){
		imgData = (unsigned char*)malloc(orgData.data.length());
		memcpy(imgData, (unsigned char*)(&(orgData.data[0])), orgData.data.length());
		m_cvSimImage = (IplImage *)imgData;
		m_cvSimImage->roi = NULL;
		m_cvSimImage->maskROI = NULL;
		m_cvSimImage->imageId = NULL;
		m_cvSimImage->tileInfo = NULL;
		m_cvSimImage->imageDataOrigin = NULL;
		m_cvSimImage->imageData = (char*)(&(imgData[sizeof(IplImage)]));
		SetData(m_cvSimImage);
	}
}

/**
 *@brief イメージデータ領域作成
 *@param width 生成する画像の幅
 *@param height 生成する画像の高さ
 */
void ImageData::CreateImage(const unsigned long width, const unsigned long height)
{
	m_width		= width;
	m_height	= height;

	DeleteImage();
	m_image = (unsigned char *)malloc( width * height );
	if(m_image == 0) {
		fprintf(stderr, "Can't malloc");
	}
	Clear(MAX_BRIGHTNESS);
}

/**
 *@brief イメージデータ領域を特定の値で埋める
 *@param value 埋め込みたいデータ(デフォルトでは0xFF)
 */
void ImageData::Clear(unsigned char value)
{
	if(m_image != 0)
		memset(m_image, value, m_width * m_height);
}

/**
 *@brief イメージデータ領域を開放する
 *
 */
void ImageData::DeleteImage()
{
	if(m_image != 0)
		free(m_image);
	m_image = 0;

	if(m_cvImage != 0)
		cvReleaseImage(&m_cvImage);
	m_cvImage = 0;

	if(m_cvSimImage)
		free(m_cvSimImage);
	m_cvSimImage = 0;
}

/**
 *@brief イメージデータを読み込み、2値画像に変換します。
 *@note OpenCVのcvLoadImageで読み込み可能な形式であれば読み込みます。
 *@param fileName 読み込むイメージファイル名
 *@param flags cvLoadImageに渡すflags値
 *@attention flagsは、本関数内でCV_LOAD_IMAGE_ANYDEPTHが設定されます。
 *@return 0:正常、-1:読み込みエラー
 */
int	ImageData::LoadImage(std::string fileName, int flags)
{
	/* イメージ領域の破棄を行っておく */
	DeleteImage();

	m_cvImage = cvLoadImage(fileName.c_str(), flags | CV_LOAD_IMAGE_ANYDEPTH);
	if(m_cvImage == 0)	return -1;
	SetData(m_cvImage);

	return 0;
}

/**
 *@brief イメージデータ形式変換
 *@param cvImage OpenCV形式データ
 *@attention 全パターンを試したわけではないので、変換出来ずに落ちる事もあるかも・・・。
 */
void ImageData::SetData(IplImage *cvImage)
{
	IplImage	*grayImage = 0;
	char		*imageData;

	/* 画像の幅・高さをとりあえず格納しておく */
	m_width		= cvImage->width;
	m_height	= cvImage->height;
	imageData	= cvImage->imageData;

	if((cvImage->width * cvImage->height) != cvImage->imageSize){
		/* イメージデータがカラー画像 or アライメント調整されている */
		/* 最終的にnChannels=1,depth=IPL_DEPTH_8U or 8S になれば良い */
		if(cvImage->nChannels != 1){
			/* カラー画像らしい */
			/* グレースケールに変換する */
			grayImage = cvCreateImage( cvGetSize(cvImage)  , IPL_DEPTH_8U, 1 );
			cvCvtColor(cvImage, grayImage, CV_BGR2GRAY);
			imageData = grayImage->imageData;
		}
		else{
			/* グレースケール or 二値画像なので、アライメント調整されているだけ */
			m_width		= cvImage->widthStep;
			m_height	= cvImage->height;
		}

	}

	m_image		= (unsigned char *)malloc(m_width * m_height);
	memcpy(m_image, imageData, m_width * m_height);
	if(grayImage != 0)	cvReleaseImage(&grayImage);
}

/**
 *@brief イメージデータの代入オペレータ
 *@attention 本オペレータでは、本クラスで独自定義しているイメージデータのみ代入され、その他2形式のイメージデータは代入されない。
 */
ImageData& ImageData::operator=(const ImageData& org)
{
	CreateImage(org.m_width, org.m_height);
	memcpy(m_image, org.m_image, org.m_width * org.m_height);
	return *this;
}
