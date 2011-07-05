#pragma once

#include "std_hdr.h"
#include "CeilingMap.h"
#include "ImageData.h"

/**
 *@brief ブロックマッチング制御クラス
 */
class BlockMatching
{
public:
	BlockMatching(void);
	~BlockMatching(void);

	void CreateBlockMap(const long center_x,
						const long center_y,
						const unsigned long image_width,
						const unsigned long image_height,
						const unsigned short block_size,
						const double Coefficient,
						const unsigned short Resolution,
						unsigned short BlackWhiteValue);
	void MapTracking(ImageData &CeilingMap, ImageData &CurPos);
	void MapTracking(CeilingMap &CeilingMap, ImageData &CurImage);
	int	SumOfAbsoluteDifference(ImageData &src, ImageData &data, unsigned long offset_x = 0, unsigned long offset_y = 0);
	SignedOdometry_st Matching(CeilingMap &CeilingMap, ImageData &CurImage, int reach);
	void Packing(ImageData &org, ImageData &packData, double theta);

private:
	void CreateBlockTable();
	void DeleteBlockTable();

	long				***m_BlockTable;	/* 画像の回転・縮小テーブル		*/
	unsigned short		m_LatticeSize;		/* 画像の最終サイズ				*/
	int					m_LatticeSize_Z;	/* 画像の360回転の分解能		*/
	int					m_Resolution;		/* 画像の360回転の分解能		*/
	unsigned short		m_BlackWhiteValue;	/* 2値化閾値					*/
};
