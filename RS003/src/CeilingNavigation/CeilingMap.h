#pragma once

#include "intellirobotStub.h"
#include "std_hdr.h"
#include "ImageData.h"

typedef struct _tag_CeilingOdometry
{
	unsigned long	m_x;
	unsigned long	m_y;
	double			m_theta;
} CeilingOdometry_st, *CeilingOdometry_pst;

typedef struct _tag_SignedOdometry
{
	long	m_x;
	long	m_y;
	double	m_theta;
} SignedOdometry_st, *SignedOdometry_pst;

/**
 *@brief 天井画像管理クラス
 */
class CeilingMap : public ImageData
{
public:
	CeilingMap(void);
	~CeilingMap(void);

	void ReadNavigationMap(std::string map_name);
	void SetLocation(unsigned long x, unsigned long y, double theta);
	void SetLocation(CeilingOdometry_st value);
	CeilingOdometry_st	GetLocation();
	void IncrementOdometry(long x, long y, double theta);
	void IncrementOdometry(SignedOdometry_st value);

	/**
	 * @brief マッチング後の対象の向きを取得する
	 * @return 対象の向き(ラジアン)
	 */
	double	GetTheta()	{ return m_CurOdometry.m_theta; }

	/**
	 * @brief マッチング後の対象のX位置を取得する
	 * @return 対象のX位置
	 */
	unsigned long	GetPosX()	{ return m_CurOdometry.m_x; }

	/**
	 * @brief マッチング後の対象のY位置を取得する
	 * @return 対象のY位置
	 */
	unsigned long	GetPosY()	{ return m_CurOdometry.m_y; }

private:
	CeilingOdometry_st	m_CurOdometry;	/* 対象の位置情報	*/
};
