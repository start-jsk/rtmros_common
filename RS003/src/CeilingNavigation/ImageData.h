#pragma once

#include <rtm/idl/BasicDataTypeSkel.h>

#include "std_hdr.h"
#include "cv.h"
#include "highgui.h"

#define MAX_BRIGHTNESS	255		/* イメージデータ初期化値	*/

using namespace RTC;

/**
 *@brief 2値イメージデータ管理クラス
 *@note 厳密には、OpenCVのデプスビットがIPL_DEPTH_8Uを扱うのでグレースケールも扱っている。
 *@attention カラーイメージを入力させる際には、事前にグレースケールに変換しておくことを推奨する。
 */
class ImageData
{
public:
	ImageData(void);					/* コンストラクタ		*/
	~ImageData(void);					/* デストラクタ			*/
	ImageData(const ImageData& src);	/* コピーコンストラクタ */

/* 公開関数 */
public:
	void CreateImage(const unsigned long width, const unsigned long height);	/* 2値イメージデータ領域作成 				*/
	void Clear(unsigned char value = MAX_BRIGHTNESS);							/* イメージデータ領域初期化 				*/
	void DeleteImage();															/* イメージデータ領域破棄					*/
	ImageData PyrDown();														/* 画像の畳み込み							*/
	void SetData(TimedOctetSeq orgData);										/* イメージデータ変換						*/
	void SetData(IplImage *cvImage);											/* OpenCV形式イメージを独自形式に変換する 	*/
	int	LoadImage(std::string fileName, int flags = CV_LOAD_IMAGE_ANYCOLOR);	/* イメージデータ読み込み					*/

	/**
	 *@brief イメージデータの幅を取得します
	 *@return イメージデータの幅
	 */
	unsigned long GetWidth()	{ return m_width; }								/* イメージ幅の取得							*/

	/**
	 *@brief イメージデータの高さを取得します
	 *@return イメージデータの高さ
	 */
	unsigned long GetHeight()	{ return m_height; }							/* イメージ高さの取得						*/

/* operator実装 */
public:
#ifdef WIN32
	/**
	 *@brief イメージデータへの配列形式アクセス
	 */
	unsigned char &operator[](size_t index)	{ return m_image[index];	}
#endif

	/**
	 *@brief イメージデータへの配列形式アクセス
	 */
	unsigned char &operator[](int index)	{ return m_image[index];	}

	/**
	 *@brief イメージデータへの2次元配列形式アクセス
	 *@param x : X座標位置
	 *@param y : Y座標位置
	 */
	unsigned char &operator()(size_t x, size_t y)	{ return m_image[x + y * m_width];	}

	/**
	 *@brief イメージデータへのポインタ取得
	 */
	operator unsigned char*()				{ return m_image;			}

	/**
	 *@brief 本クラスの代入処理
	 */
	ImageData& operator=(const ImageData& org);

private:
	unsigned long	m_width;		/* イメージデータ幅						*/
	unsigned long	m_height;		/* イメージデータ高さ					*/
	unsigned char	*m_image;		/* イメージデータ領域					*/
	IplImage		*m_cvImage;		/* OpenCV形式イメージデータ領域			*/
	IplImage		*m_cvSimImage;	/* 擬似OpenCV形式イメージデータ領域		*/
};
