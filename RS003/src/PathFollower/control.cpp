/*high level I/O*/
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <strings.h>
#include <pthread.h>

/*low level I/O*/
#include <fcntl.h>
#include <sys/time.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>

/*serial*/
#include <sys/termios.h>

#include "PathFollower.h"
#include "PathType.h"

#define SIGN(x)	((x < 0) ? -1 : 1)

/*-PI < theta < PIに調整する*/
double trans_q(double theta){
  while(theta > M_PI)theta -= 2.0*M_PI;
  while(theta < -M_PI)theta += 2.0*M_PI;
  return theta;
}

/*円弧追従*/
double PathFollower::circle_follow(double x,double y, double theta,
		     double cx, double cy, double cradius,
		     double v_max){
  double d,q,r,ang;
  
  r = sqrt((x - cx)*(x -cx) +(y -cy)*(y -cy));
  
  ang = atan2((y - cy), (x - cx));
  ang = trans_q(ang);

  // レギュレータ問題に変換
  d = fabs(cradius) - r;
  q = trans_q(theta - (ang + SIGN(cradius) * (M_PI / 2.0)));
  
  return regurator(d, q, cradius, v_max);
}

/*直線追従*/
double PathFollower::line_follow(double x,double y,double theta,
		   double cx, double cy, double ctheta,
		   double v_max){
  double d,q;
  
  d = -(x-cx)*sin(ctheta) + (y-cy)*cos(ctheta);
  
  /*yukkuri*/
  /*  if(d > 0.5){
    d -= 0.5;
  }else if(d < -0.5){
    d += 0.5;
  }else {
    d = 0;
    }*/ 

  q = theta - ctheta;
  q = trans_q(q);

  return regurator(d, q, 1000000, v_max);
}

/*軌跡追従レギュレータ*/
double PathFollower::regurator(double d, double q, double r,  double v_max){
  double nv,nw;
  double v,w;
  double cd;

  nv = target_v;
  nw = target_w;
  v = v_max - SIGN(v_max) * m_line_C1 * fabs(nw);
  
  cd = d;
  if(cd >   m_line_Dist)cd = m_line_Dist;
  if(cd < - m_line_Dist)cd = - m_line_Dist;
  w = nw -  m_control_cycle*
    (SIGN(r)*SIGN(nv) * m_line_K1*cd + m_line_K2*q + m_line_K3*nw);
  
  /*FF*/
  //  if(fabs(r)>0.1 )
  //  w += 2*nv / r ; 

  v = v_max;
  /*#ifdef UTA //UTA Magic
  v_ref.w = (vel.w + v_ref_old.w) / 2.0
     - period * (SIGN(r) * SIGN(vel.v) * R.K1 * y + R.K2 * q + R.K3 * vel.w);
#endif
  */

  robot_speed_smooth(v, w);  
  return d;
}

/*回転*/
double PathFollower::spin(double theta, double target_theta, double w_max){
  double q, w_limit;
  double w;
  
  q = target_theta - theta;
  q = trans_q(q);
  
  /*停止するのに限界の速度を計算*/
  w_limit = sqrt(0.1*2*m_max_acc_w*fabs(q)/m_control_cycle);

  if(w_max < w_limit){
    w = SIGN(q)*w_max;
  }else{
    w = SIGN(q)*w_limit;
    if(fabs(w) < M_PI/90.0)w = M_PI/90.0*SIGN(q);
  }
  
  robot_speed_smooth(0, w);  
  return fabs(q);
}

/*点までの距離*/
double PathFollower::dist_pos(double x,double y, double tx, double ty){
  double r;
  r = sqrt((x -tx)*(x -tx)+(y -ty)*(y -ty));
  
  return r;
}

/*直線の端点まで移動し止まる*/
int PathFollower::to_point(double x,double y,double theta,
			   double tx, double ty, double ttheta,double max_vel){
  double dist,a ;
  double vel;
  int over;

  dist = dist_pos(x,y, tx, ty);

  a = (x -tx)*cos(ttheta) + (y-ty)*sin(ttheta);
  over = 0;
  if(a > 0){
    vel = 0;
    over = 3;
  }else if(dist > max_vel){
    vel = max_vel;
  }else if(dist > 0.05){
    over = 1;
    vel = dist;  
  }else{
    over = 2;
    vel = dist;
  }

  line_follow(x,y,theta, tx,ty,ttheta,vel);
  return over;
}

/*velocity clippung*/
int PathFollower::robot_speed_smooth(double v, double w){
  int limit;

  limit = 15;

  if(v > m_max_v){
    v = m_max_v;
  }else if(v < -m_max_v){
    v = -m_max_v;
  }else{
    limit -=1;
  }

  if(v > target_v + m_max_acc_v){
    v= target_v+ m_max_acc_v;
  }else if(v < target_v - m_max_acc_v){
    v= target_v - m_max_acc_v;
  }else{
    limit-= 2;
  }


  if(w > m_max_w){
    w = m_max_w;
  }else if(w < -m_max_w){
    w = -m_max_w;
  }else{
    limit -=4;
  }

  if(w > target_w + m_max_acc_w){
    w= target_w + m_max_acc_w;
  }else if(w < target_w -  m_max_acc_w){
    w= target_w - m_max_acc_w;
  }else{
    limit -= 8;
  }

  target_v = v;
  target_w = w;

  return limit;
}
double get_time(void)
{
  struct timeval current;

  gettimeofday(&current, NULL); 
  
  return  current.tv_sec + current.tv_usec/1000000.0;   
}

/*追従軌跡に応じた処理*/
//void* PathFollower::run_control(void *){
void PathFollower::path_following(double x,double y,double theta){
  static double before_time;
  double now_time;

  //  while(do_control){
  now_time = get_time();
  if(!m_positionIn.isEmpty())m_positionIn.read();
  
  if(now_time > before_time){/*20ms毎？*/
    
    /*パラメータの変更がおこらないようにブロック*/
    //pthread_mutex_lock(&mutex);
    
    before_time = now_time;      
    
    /*走行状態に応じた処理*/
    switch(run_mode){
    case RUN_STOP://ストップする（スピードを0にする）
      robot_speed_smooth(0,0);
      break;
    case RUN_VEL://速度角速度指定
      robot_speed_smooth(path_v, path_w);
      break;
    case RUN_LINEFOLLOW: //直線追従
      line_follow(x,y,theta,
		  path_x, path_y, path_theta,path_v);
      break;
    case RUN_TO_POINT: //短辺への移動
      to_point(x,y,theta,
	       path_x, path_y, path_theta,path_v);
      break;
    case RUN_CIRCLEFOLLOW: //円弧追従
      circle_follow(x, y, theta,
		    path_x, path_y, path_d,path_v);
      break;
    case RUN_SPIN://回転
      spin(theta, path_theta, path_w);
      break;
    }
    
    /*保護終わり*/
    //pthread_mutex_unlock(&mutex);
    
  }else{
    //    usleep(1000);
  }
  //}
}


void PathFollower::initPathFollower(void){
  do_control = 1;
  run_mode = RUN_STOP;

  target_v = 0;
  target_w = 0;

  /*  pthread_mutex_init(&mutex, NULL);
  printf("starting control thread.\n");

  if(pthread_create(&run_thread, NULL, run_control, NULL)!= 0){
    fprintf(stderr, "Can't create command thread\n");
  }
  */
}


void PathFollower::finalizePathFollower(void){
  /*
  do_control = 0;
  printf("Control thread is stopping.\n");
  pthread_join(run_thread, NULL);
  */}
