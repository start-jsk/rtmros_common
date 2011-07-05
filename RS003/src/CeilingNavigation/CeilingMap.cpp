#include "CeilingMap.h"

/**
 * @brief コンストラクタ
 * @note 処理は行っていない
 */
CeilingMap::CeilingMap(void)
{
}

/**
 * @brief デストラクタ
 * @note 処理は行っていない
 */
CeilingMap::~CeilingMap(void)
{
}

/**
 * @brief ナビゲーションマップの読み込み
 * @param map_name 天井画像ファイル名
 */
void CeilingMap::ReadNavigationMap(std::string map_name)
{
	LoadImage(map_name);
}

/**
 * @brief 座標点の設定を行う
 * @param x X座標
 * @param y Y座標
 * @param theta 向き(ラジアン)
 */
void CeilingMap::SetLocation(unsigned long x, unsigned long y, double theta)
{
	m_CurOdometry.m_x		= x;
	m_CurOdometry.m_y		= y;
	m_CurOdometry.m_theta	= theta;
}

/**
 * @brief 座標点の設定を行う
 * @param value 座標情報
 * @note SetLocation(unsigned long x, unsigned long y, double theta)の呼び出しを行う
 */
void CeilingMap::SetLocation(CeilingOdometry_st value)
{
	SetLocation(value.m_x, value.m_y, value.m_theta);
}

/**
 * @brief 現在の座標情報を取得する。
 * @param 座標情報
 */
CeilingOdometry_st	CeilingMap::GetLocation()
{
	return m_CurOdometry;
}

/**
 * @brief 座標情報を更新する
 * @note 本関数は、現在の情報に対して、パラメタで指定された情報を加算する。
 * @param x X位置
 * @param y Y位置
 * @param theta 向き(ラジアン)
 */
void CeilingMap::IncrementOdometry(long x, long y, double theta)
{
	m_CurOdometry.m_x += x;
	m_CurOdometry.m_y += y;
	m_CurOdometry.m_theta += theta;

	/* ±１８０度に収まるように調整する */
	while (m_CurOdometry.m_theta > M_PI)
		m_CurOdometry.m_theta = -M_PI*2 + m_CurOdometry.m_theta;
	while (m_CurOdometry.m_theta < -M_PI)
		m_CurOdometry.m_theta =  M_PI*2 + m_CurOdometry.m_theta;
}

/**
 * @brief 座標情報を更新する
 * @param value 座標情報
 * @note IncrementOdometry(long x, long y, double theta)の呼び出しを行う
 */
void CeilingMap::IncrementOdometry(SignedOdometry_st value)
{
	IncrementOdometry(value.m_x, value.m_y, value.m_theta);
}
