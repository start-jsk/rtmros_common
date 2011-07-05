// -*- C++ -*-
/*!
 * @file  LocalizeCenter.cpp
 * @brief ${rtcParam.description}
 * @date $Date$
 *
 * $Id$
 */

#include "LocalizeCenter.h"

// Module specification
// <rtc-template block="module_spec">
static const char* localizecenter_spec[] =
  {
    "implementation_id", "LocalizeCenter",
    "type_name",         "LocalizeCenter",
    "description",       "${rtcParam.description}",
    "version",           "1.0.0",
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    "conf.default.startX", "3.24",
    "conf.default.startY", "7.81",
    "conf.default.startTheta", "-1.57",
		"conf.default.cycle", "20",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
LocalizeCenter::LocalizeCenter(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_odmIn("dp_odmIn", m_odm),
    m_ceilIn("dp_ceilIn", m_ceil),
    //m_in2In("dp_in2", m_in2),
    m_dp_out0Out("dp_out0", m_dp_out0)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
LocalizeCenter::~LocalizeCenter()
{
}



RTC::ReturnCode_t LocalizeCenter::onInitialize()
{
  bindParameter("startX", m_start_x, "3.29");
  bindParameter("startY", m_start_y, "7.70");
  bindParameter("startTheta", m_start_theta, "-1.571");
	bindParameter("cycle", m_cycle, "20");	
	
	start_x_prev = m_start_x;
	start_y_prev = m_start_y;
	start_theta_prev = m_start_theta;
	
	pos_init_cnt = 0;

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("dp_odmIn", m_odmIn);
  addInPort("dp_ceilIn", m_ceilIn);
  //addInPort("dp_in2", m_in2In);
  
  // Set OutPort buffer
  addOutPort("dp_out0", m_dp_out0Out);
  
	//初期値設定
	m_odm.data.position.x = m_ceil.data.position.x = m_avPos.x = m_start_x;
	m_odm.data.position.y = m_ceil.data.position.y = m_avPos.y = m_start_y;
	m_odm.data.heading = m_ceil.data.heading = m_avPos.theta = m_start_theta;

	//重みの設定
	m_odmWgt = 19.0;
	m_ceilWgt = 1.0;
	m_totalWgt = m_odmWgt + m_ceilWgt;
	counter = 0;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t LocalizeCenter::onActivated(RTC::UniqueId ec_id)
{
	//スタート位置に関するコンフィギュレーションパラメータが変更されていたらそれを反映させる
	if( ! ( start_x_prev == m_start_x && start_y_prev == m_start_y && start_theta_prev == m_start_theta ) )
	{
		pos_init_cnt = 0;
		start_x_prev = m_start_x;
		start_y_prev = m_start_y;
		start_theta_prev = m_start_theta;
		
		m_odm.data.position.x = m_ceil.data.position.x = m_avPos.x = m_start_x;
		m_odm.data.position.y = m_ceil.data.position.y = m_avPos.y = m_start_y;
		m_odm.data.heading = m_ceil.data.heading = m_avPos.theta = m_start_theta;

	}
	
	m_dp_out0.data.position.x = m_avPos.x;
	m_dp_out0.data.position.y = m_avPos.y;
	m_dp_out0.data.heading = m_avPos.theta;

	//最初に一回出力ポートから送信
	m_dp_out0Out.write();

  return RTC::RTC_OK;
}


RTC::ReturnCode_t LocalizeCenter::onExecute(RTC::UniqueId ec_id)
{
	int odmFlag=0, ceilFlag=0;

	//onActivateで位置を変えるときの処理．はじめに数回位置推定モジュールに
	//変更後の位置情報を流し込む．
	if(pos_init_cnt < 20){
		m_dp_out0Out.write();
		pos_init_cnt ++;
	}
	else{
		if(m_odmIn.isNew()){
			while(!m_odmIn.isEmpty())//H.T 20100822 リングバッファの最新の値を読む
			{
				m_odmIn.read();
				odmFlag = 1;
			}
		}
		if(m_ceilIn.isNew()){
			while(!m_ceilIn.isEmpty())//H.T 20100822 リングバッファの最新の値を読む
			{
				m_ceilIn.read();
				ceilFlag = 1;
			}

			m_avPos.x = ( m_odmWgt * m_odm.data.position.x + m_ceilWgt * m_ceil.data.position.x ) / m_totalWgt;
			m_avPos.y = ( m_odmWgt * m_odm.data.position.y + m_ceilWgt * m_ceil.data.position.y ) / m_totalWgt;

			//if( m_odm.theta * m_ceil.theta < 0 && abs( m_ceil.theta ) > 3.0 * M_PI / 4.0)
			if( m_odm.data.heading * m_ceil.data.heading < 0 && abs( m_ceil.data.heading ) > 3.0 * M_PI / 4.0)
			{ //π側と-π側で打ち消してしまいそうな時
				if( m_odm.data.heading > m_ceil.data.heading)
					m_ceil.data.heading = m_ceil.data.heading + 2 * M_PI;
				else
					m_odm.data.heading = m_odm.data.heading + 2 * M_PI;
			}
		
				m_avPos.theta = ( m_odmWgt * m_odm.data.heading + m_ceilWgt * m_ceil.data.heading ) / m_totalWgt;
			

			if (m_avPos.theta > M_PI)		// 180度を越えている場合、マイナスに変更する
				m_avPos.theta = -M_PI * 2 + m_avPos.theta;
			else if (m_avPos.theta < -M_PI)	// -180を下回る場合、プラスに変更する
				m_avPos.theta = M_PI * 2 + m_avPos.theta;

			m_dp_out0.data.position.x = m_avPos.x;
			m_dp_out0.data.position.y = m_avPos.y;
			m_dp_out0.data.heading = m_avPos.theta;

			m_dp_out0Out.write();
			printf("%d %d : X=%f Y=%f Th=%f\n",odmFlag,ceilFlag,m_avPos.x,m_avPos.y,m_avPos.theta);
			counter=0;
		}

		else{//天井ナビゲーションからのデータが来ていないとき
			ceilFlag = 0;
			m_avPos.x = m_odm.data.position.x;
			m_avPos.y = m_odm.data.position.y;
			m_avPos.theta = m_odm.data.heading;
			

			if (m_avPos.theta > M_PI)		// 180度を越えている場合、マイナスに変更する
				m_avPos.theta = -M_PI * 2 + m_avPos.theta;
			else if (m_avPos.theta < -M_PI)	// -180を下回る場合、プラスに変更する
				m_avPos.theta = M_PI * 2 + m_avPos.theta;

			m_dp_out0.data.position.x = m_avPos.x;
			m_dp_out0.data.position.y = m_avPos.y;
			m_dp_out0.data.heading = m_avPos.theta;
			counter++;
		
			if(counter%m_cycle ==0){
				counter = 0;
				m_dp_out0Out.write();
				printf("%d %d : X=%f Y=%f Th=%f\n",odmFlag,ceilFlag,m_avPos.x,m_avPos.y,m_avPos.theta);
			}
		}
	}


  return RTC::RTC_OK;
}

extern "C"
{
 
  void LocalizeCenterInit(RTC::Manager* manager)
  {
    coil::Properties profile(localizecenter_spec);
    manager->registerFactory(profile,
                             RTC::Create<LocalizeCenter>,
                             RTC::Delete<LocalizeCenter>);
  }
  
};


