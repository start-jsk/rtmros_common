#include "BlockMatching.h"
#include "CeilingMap.h"
#ifdef _DEBUG
#ifdef WIN32
#include <crtdbg.h>
#endif
#endif

#ifdef _USE_SHOW_MAP
#include "Common/sharedmem.h"
extern robot_status *rstat;
#endif

#define SEARCH_RANGE	4
#define SIZE_HALF	(block_size / 2) //モジュール間共通の値．何とかすべし
#define ANGLE_MAX	(m_Resolution * 2 + 1)

/**
 *@brief コンストラクタ
 */
BlockMatching::BlockMatching(void) : m_BlockTable(0)
{
}

/**
 *@brief デストラクタ
 *@note 画像の回転・縮小テーブル領域を破棄する
 */
BlockMatching::~BlockMatching(void)
{
	if(m_BlockTable != 0)
		DeleteBlockTable();
}

/**
 * @brief 回転・縮小配列を作成する
 * @param center_x 画像回転中心X座標。-1の場合は画像の中心を使用する。(デフォルト-1)
 * @param center_y 画像回転中心Y座標。-1の場合は画像の中心を使用する。(デフォルト-1)
 * @param image_width 入力画像幅
 * @param image_height 入力画像高さ
 * @param block_size 最終的な画像サイズ(縦・横サイズ)
 * @param Coefficient 画像縮小用係数
 * @param Resolution 回転分解能(何度刻みで回転画像を生成するか)
 * @param BlackWhiteValue 2値化の閾値
 *
 */
void BlockMatching::CreateBlockMap(const long center_x,
						   const long center_y,
						   const unsigned long image_width,
						   const unsigned long image_height,
						   const unsigned short block_size,
						   const double Coefficient,
						   const unsigned short Resolution,
						   const unsigned short BlackWhiteValue)
{
	int		i;
	int		x,	y;
	int		x2,	y2;
	int		sx,	sy;
	int		degree_min, degree_max;
	double	cx,	cy, dx1, dy1;
	unsigned long width, height;

	/*
		中心点が定義されていない場合は画像データの中心点を採用する
		(最終的な中心座標とするため1/4サイズの中心にする)
	*/

	width = image_width / 2;
	if(center_x == -1)
		cx = width / 2;
	else
		cx = center_x;	/* カメラセンター設定 */

	height = image_height / 2;
	if(center_y == -1)
		cy = height / 2;
	else
		cy = center_y;	/* カメラセンター設定 */

	sx = cx - (int)((double)SIZE_HALF / Coefficient);
	sy = cy - (int)((double)SIZE_HALF / Coefficient);

	m_Resolution = 360 / 2 / Resolution;
	degree_max = m_Resolution; //回転角のインデックスの数(正の方向)
	degree_min = m_Resolution * -1;//回転角のインデックスの数(負の方向)

	m_LatticeSize	= block_size;
	m_BlackWhiteValue = BlackWhiteValue;

	CreateBlockTable();

	for (y = 0; y < block_size; y++){
		for (x = 0; x < block_size; x++){
			for (i = degree_min; i <= degree_max; i++){
				x2 = sx + ( (double)((double)( x - SIZE_HALF )) * cos((double)i * M_PI / (double)m_Resolution) - 
							(double)((double)( y - SIZE_HALF )) * sin((double)i * M_PI / (double)m_Resolution) + (double)SIZE_HALF ) / Coefficient;

				y2 = sy + ( (double)((double)( x - SIZE_HALF )) * sin((double)i * M_PI / (double)m_Resolution) +
							(double)((double)( y - SIZE_HALF )) * cos((double)i * M_PI / (double)m_Resolution) + (double)SIZE_HALF ) / Coefficient;

/*				/\* 離散座標に変換にしてからでないと... *\/ */

				dx1 = x2;
				dy1 = y2;

				if (dx1 <   0 )			  dx1 = 0;
				if (dx1 > (width - 1) )	  dx1 = (width - 1);
				if (dy1 <   0 )			{ dy1 = 0;					dx1=0; }
				if (dy1 > (height - 1) ){ dy1 = (height - 1); dx1=0; }

				m_BlockTable[ i + m_Resolution][x][y] = (int)((int)dx1 + (int)dy1 * (image_width / 2));
			}
		}
	}
}

/**
 *@brief CreateBlockMapのblock_sizedeおよびResolutionで指定された大きさの配列を生成する。
 */
void BlockMatching::CreateBlockTable()
{
	m_BlockTable = new long**[ANGLE_MAX];
	for(int cnt_z = 0; cnt_z < ANGLE_MAX ; cnt_z++){
		m_BlockTable[cnt_z] = new long*[m_LatticeSize];
		for(int cnt_x_y = 0; cnt_x_y < m_LatticeSize; cnt_x_y++){
			m_BlockTable[cnt_z][cnt_x_y] = new long[m_LatticeSize];
		}
	}
}

/**
 *@brief CreateBlockTable()で生成した領域を破棄する。
 */
void BlockMatching::DeleteBlockTable()
{
	for(int cnt_z = 0; cnt_z < ANGLE_MAX ; cnt_z++){
		for(int cnt_x_y = 0; cnt_x_y < m_LatticeSize; cnt_x_y++){
			delete[] m_BlockTable[cnt_z][cnt_x_y];
		}
		delete[] m_BlockTable[cnt_z];
	}
	delete[] m_BlockTable;
	m_BlockTable = 0;
}

/**
 * @brief マップ追跡を行う。
 * @param CeilingMap マッチングを行うベース画像への参照
 * @param CurImage 現在のイメージへの参照
 *
 */
void BlockMatching::MapTracking(CeilingMap &CeilingMap, ImageData &CurImage)
{
	ImageData		capImage;
	SignedOdometry_st	odometry;
#ifdef _USE_SHOW_MAP
	int				x,		y;
#endif
	static int reach = 0; //マッチングの探索範囲を調整するパラメータと思われるがとりあえず不使用に

	/* マップマッチング */
	/* マッチングするには、キャプチャ画像を1/4スケールに縮める必要がある */
	capImage = CurImage.PyrDown();

	odometry = Matching(CeilingMap, capImage, reach);
	CeilingMap.IncrementOdometry(odometry);

#ifdef _USE_SHOW_MAP
	ImageData		PackImage;
	PackImage.CreateImage(m_LatticeSize, m_LatticeSize);
	Packing(capImage, PackImage, 0/*CeilingMap.GetTheta()*/);//ここで画像を回転している

	for (y = 0; y < m_LatticeSize; y++) {
		for (x = 0; x < m_LatticeSize; x++) {
			rstat->buf[x+y*m_LatticeSize] = (PackImage[x+y*m_LatticeSize]);
		}
	}  
	rstat->x = CeilingMap.GetPosX();
	rstat->y = CeilingMap.GetPosY();
	rstat->r = CeilingMap.GetTheta();
#endif
}

/**
 *@brief 画像マッチングを行う。
 *@param CeilingMap マッチングを行うベース画像への参照
 *@param CurImage 現在画像への参照
 *@param reach マッチング範囲補正値
 */
SignedOdometry_st BlockMatching::Matching(CeilingMap &CeilingMap, ImageData &CurImage, int reach)
{
	SignedOdometry_st	result;
	ImageData		PackImage;
	int				min;
	int				i,		s;
	int				x,		y;

	result.m_x = 0;
	result.m_y = 0;
	result.m_theta = 0.0;

	PackImage.CreateImage(m_LatticeSize, m_LatticeSize);
	Packing(CurImage, PackImage, CeilingMap.GetTheta());

	// 評価基準値を求めておく・・・現在の位置より良い値を探すため
	min = SumOfAbsoluteDifference(CeilingMap, PackImage, CeilingMap.GetPosX(), CeilingMap.GetPosY());

	//角度方向探索 ±m_Resolution * SEARCH_RANGE
	for ( i = -(SEARCH_RANGE + reach); i <= (SEARCH_RANGE + reach); i++) { 

		Packing(CurImage, PackImage, CeilingMap.GetTheta() + (double)( i * M_PI / m_Resolution ));
		//X方向探索 ±SEARCH_RANGE
		for ( x = -(SEARCH_RANGE + reach); x <= (SEARCH_RANGE + reach); x++) {

			//Y方向探索 ±SEARCH_RANGE
			for ( y = -(SEARCH_RANGE + reach); y <= (SEARCH_RANGE + reach); y++) {
				s = SumOfAbsoluteDifference(CeilingMap, PackImage,  CeilingMap.GetPosX() + x, CeilingMap.GetPosY() + y);

				if (s < min) {
					//現在地の更新
					min = s;
					result.m_x = x;
					result.m_y = y;
					result.m_theta = i;
				}
			}
		}
	}
	result.m_theta = result.m_theta * M_PI / m_Resolution;

	return result;
}


/**
 *@brief		輝度差の総和(SAD; Sum of Absolute Difference)を計算する
 *@attention	比較は(標準では)64×64で行うので、画像データは64×64にしておく必要がある
 *@note			2つの画像の同一座標点の差分を積算する。
 *@param		src マッチングを行うベース画像の参照
 *@param		data 64×64縮小済み現在画像の参照
 *@param		increment_x ベース画像原点からのオフセット
 *@param		increment_y ベース画像原点からのオフセット
 */
int BlockMatching::SumOfAbsoluteDifference(ImageData &src, ImageData &data, unsigned long offset_x, unsigned long offset_y)
{
	int cell_x, cell_y;
	int result, diff;

	result = 0;
	if ( src.GetWidth() == 0 || src.GetHeight() == 0 || data.GetWidth() == 0 || data.GetHeight() == 0 ) {
	  fprintf(stderr, "Warning : %ssrc or data image is 0 size\n", __PRETTY_FUNCTION__);
	  return result;
	}
	for( cell_y = 0; cell_y < m_LatticeSize; cell_y++) {
		for( cell_x = 0; cell_x < m_LatticeSize; cell_x++) {
			diff = src(cell_x + offset_x, cell_y + offset_y) - data(cell_x, cell_y);
			if ( diff < 0)
				result -= diff;
		}
	}
	return result;
}

/**
 *@brief 画像を64×64に縮小する
 *@param org 縮小元画像の参照



 *@param packData 64×64縮小データ格納先の参照
 *@param theta 画像の回転角度
 */
void BlockMatching::Packing(ImageData &org, ImageData &packData, double theta)
{
	int x, y;
	int rotation;

	if (theta > M_PI)		/* 180度を越えている場合、マイナスに変更する */
		theta = -M_PI * 2 + theta;
	else if (theta < -M_PI)	/* -180を下回る場合、プラスに変更する */
		theta = M_PI * 2 + theta;

	rotation = (int)(theta / ( M_PI / m_Resolution ) ) + m_Resolution;	/* ラジアン->度 変換 */

	for (x=0; x < m_LatticeSize; x++) {
		for (y=0; y < m_LatticeSize; y++) {
			if (m_BlockTable[rotation][x][y] >= 0 && m_BlockTable[rotation][x][y] < org.GetWidth() * org.GetHeight())
				packData[x+y*m_LatticeSize] = org[m_BlockTable[rotation][x][y]];	/* 回転した場合の座標位置にあるデータを取り出す */
			else
				packData[x+y*m_LatticeSize] = 0;

			if (packData[x+y*m_LatticeSize] > m_BlackWhiteValue )	/* 2値化の閾値判定を行い、白/黒に変更する */
				packData[x+y*m_LatticeSize] = MAX_BRIGHTNESS;
			else
				packData[x+y*m_LatticeSize] = 0;
		}
	}
}
