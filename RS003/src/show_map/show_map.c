#include <stdio.h>
#ifndef WIN32
#include <unistd.h>
#endif

#ifdef WIN32
#define _USE_MATH_DEFINES
#endif
#include <math.h>

#include <cv.h>
#include <highgui.h>

#include "../Common/sharedmem.h"

#ifdef WIN32
#include "../Common/Linux4Win.h"
#endif

#include "define.h"


#define MAP_WINDOW "ceil map"
//#define BIG_WINDOW "Localization" //これを定義するとceil mapと同じものがscale倍されて表示される


static void on_mouse(int event, int x, int y, int flags, void* param);

//共有メモリから読み取るロボットの状態 ../Common/sharedmem.hで定義
static robot_status *rstat;

int main (int argc, char **argv )
{
#ifdef BIG_WINDOW
  double scale = 3.0;
#endif
  int key = 0;
  IplImage *map_img;//天井地図画像
  IplImage *output_img;//出力画像（天井地図画像と同サイズ）
  IplImage *capture_img;//共有メモリから読んだカメラ画像
  IplImage *capture_tmp;//共有メモリから読んだカメラ画像
#ifdef SAVE_TRACK
  char fname[256];
  unsigned int count = 0;
#endif
  CvRect roi;

  //共有メモリ初期化  書き込み側のRTC(CeilingNavigationLinux)とSHM_KEYを合わせておくこと．
  rstat = (robot_status *)InitSharedMem(SHM_KEY, sizeof(robot_status));

  //地図となる天井画像の読み込み
  map_img = cvLoadImage (MAP_SHOW, CV_LOAD_IMAGE_COLOR /* CV_LOAD_IMAGE_GRAYSCALE */);//define.hで地図画像を定義
  if (!map_img)
    {//読み込みエラー処理
      fprintf(stderr, "%s: LOAD ERROR\n", MAP_SHOW);
      return 1;
    }
  
  //地図画像のサイズ確認表示
  printf("mapsize: x=%3d y=%3d\n",map_img->width,map_img->height);

  //各種イメージデータのクリエイト
  output_img = cvCreateImage( cvSize(map_img->width, map_img->height), IPL_DEPTH_8U, 3);
  capture_img = cvCreateImage( cvSize(TEMPLATE_SIZE, TEMPLATE_SIZE), IPL_DEPTH_8U, 1);//こいつだけ白黒画像であることに注意
  capture_tmp = cvCreateImage( cvSize(TEMPLATE_SIZE, TEMPLATE_SIZE), IPL_DEPTH_8U, 3);//TEMPLATE_SIZEはdefine.hで定義

#ifdef BIG_WINDOW
  IplImage *big_img = cvCreateImage (cvSize ((int)(output_img->width*scale),(int)(output_img->height*scale)), output_img->depth, output_img->nChannels);
  cvNamedWindow (BIG_WINDOW, CV_WINDOW_AUTOSIZE);
#endif

  cvNamedWindow (MAP_WINDOW, CV_WINDOW_AUTOSIZE);
  cvNamedWindow("CameraWindow", 0);
  cvSetMouseCallback(MAP_WINDOW, on_mouse, 0); //MAP_WINDOWはマウスクリックに対応

  while(1)
    {
      cvCopy (map_img, output_img,NULL);//地図画像を出力画像にコピー
      //共有メモリのカメラ画像をcapture_imgにコピー
      memcpy (capture_img->imageData, rstat->buf, sizeof(unsigned char)*TEMPLATE_SIZE*TEMPLATE_SIZE);
      //      cvConvertImage( capture_img, output_img, 0);
      //白黒画像をカラー画像に変換
      cvConvertImage( capture_img, capture_tmp, 0);
      
      //出力画像の左上にcaputure_img(capture_tmp)を表示させる処理
      roi = cvRect (0, 0, capture_img->width, capture_img->height);
      cvSetImageROI (output_img, roi);
      cvCopy (capture_tmp, output_img, 0);
      cvResetImageROI(output_img);
      //左上の表示領域に枠をつける
      cvRectangle ( output_img, cvPoint(roi.x,roi.y), cvPoint(roi.x+roi.width,roi.y+roi.height), CV_RGB(255,0,0), 1, 4, 0 );

#ifdef _DEBUG
      printf("%d, %d, %f ( %d )\n", rstat->x, rstat->y, rstat->r, rstat->s);
#endif

      //以下，ロボットの位置・方位角・マッチした画像枠の描画
      // robot point
      cvCircle ( map_img, 
		 cvPoint( (int)(rstat->x+TEMPLATE_SIZE/2),
			  (int)(rstat->y+TEMPLATE_SIZE/2) ),
		 1, CV_RGB(255,0,0), -1, 4, 0 );

      // robot point
/*      cvCircle ( map_img, 
		 cvPoint( (int)(rstat->x+TEMPLATE_SIZE),
			  (int)(rstat->y+TEMPLATE_SIZE) ),
		 1, CV_RGB(255,0,0), -1, 4, 0 );*/

      // robot point
/*      cvCircle ( map_img, 
		 cvPoint( (int)(rstat->x+TEMPLATE_SIZE/2),
			  (int)(rstat->y) ),
		 1, CV_RGB(255,0,0), -1, 4, 0 );*/

      // deg
      cvLine ( output_img,
	       cvPoint( (int)(rstat->x+TEMPLATE_SIZE/2),
			(int)(rstat->y+TEMPLATE_SIZE/2) ),
	       cvPoint( (int)(rstat->x+TEMPLATE_SIZE/2+sin(rstat->r)*10),
			(int)(rstat->y+TEMPLATE_SIZE/2+cos(rstat->r)*10) ),
	       CV_RGB(255,140,0), 1, CV_AA, 0 );

/*
      // deg
      cvLine ( output_img,
	       cvPoint( (int)(rstat->x+TEMPLATE_SIZE),
			(int)(rstat->y+TEMPLATE_SIZE) ),
	       cvPoint( (int)(rstat->x+TEMPLATE_SIZE+sin(rstat->r)*10),
			(int)(rstat->y+TEMPLATE_SIZE+cos(rstat->r)*10) ),
	       CV_RGB(255,140,0), 1, CV_AA, 0 );*/

      // deg
/*      cvLine ( output_img,
	       cvPoint( (int)(rstat->x+TEMPLATE_SIZE/2),
			(int)(rstat->y) ),
	       cvPoint( (int)(rstat->x+TEMPLATE_SIZE/2+sin(rstat->r)*10),
			(int)(rstat->y+cos(rstat->r)*10) ),
	       CV_RGB(255,140,0), 1, CV_AA, 0 );*/

      // detect rect
      cvLine ( output_img,
	       cvPoint( (int)(rstat->x + TEMPLATE_SIZE/2 + TEMPLATE_SIZE/2*(cos(-rstat->r) - sin(-rstat->r)) ),
			(int)(rstat->y + TEMPLATE_SIZE/2 + TEMPLATE_SIZE/2*(sin(-rstat->r) + cos(-rstat->r)) ) ),
	       cvPoint( (int)(rstat->x + TEMPLATE_SIZE/2 + TEMPLATE_SIZE/2*(-cos(-rstat->r) - sin(-rstat->r)) ),
			(int)(rstat->y + TEMPLATE_SIZE/2 + TEMPLATE_SIZE/2*(-sin(-rstat->r) + cos(-rstat->r)) ) ),
	       CV_RGB(255,255,0), 1, CV_AA, 0 );
      cvLine ( output_img,
	       cvPoint( (int)(rstat->x + TEMPLATE_SIZE/2 + TEMPLATE_SIZE/2*(cos(-rstat->r) - sin(-rstat->r)) ),
			(int)(rstat->y + TEMPLATE_SIZE/2 + TEMPLATE_SIZE/2*(sin(-rstat->r) + cos(-rstat->r)) ) ),
	       cvPoint( (int)(rstat->x + TEMPLATE_SIZE/2 + TEMPLATE_SIZE/2*(cos(-rstat->r) + sin(-rstat->r)) ),
			(int)(rstat->y + TEMPLATE_SIZE/2 + TEMPLATE_SIZE/2*(sin(-rstat->r) - cos(-rstat->r)) ) ),
	       CV_RGB(255,255,0), 1, CV_AA, 0 );
      cvLine ( output_img,
	       cvPoint( (int)(rstat->x + TEMPLATE_SIZE/2 + TEMPLATE_SIZE/2*(cos(-rstat->r) + sin(-rstat->r)) ),
 			(int)(rstat->y + TEMPLATE_SIZE/2 + TEMPLATE_SIZE/2*(sin(-rstat->r) - cos(-rstat->r)) ) ),
	       cvPoint( (int)(rstat->x + TEMPLATE_SIZE/2 + TEMPLATE_SIZE/2*(-cos(-rstat->r) + sin(-rstat->r)) ),
			(int)(rstat->y + TEMPLATE_SIZE/2 + TEMPLATE_SIZE/2*(-sin(-rstat->r) - cos(-rstat->r)) ) ),
	       CV_RGB(255,255,0), 1, CV_AA, 0 );
      cvLine ( output_img,
	       cvPoint( (int)(rstat->x + TEMPLATE_SIZE/2 + TEMPLATE_SIZE/2*(-cos(-rstat->r) - sin(-rstat->r)) ),
			(int)(rstat->y + TEMPLATE_SIZE/2 + TEMPLATE_SIZE/2*(-sin(-rstat->r) + cos(-rstat->r)) ) ),
	       cvPoint( (int)(rstat->x + TEMPLATE_SIZE/2 + TEMPLATE_SIZE/2*(-cos(-rstat->r) + sin(-rstat->r)) ),
			(int)(rstat->y + TEMPLATE_SIZE/2 + TEMPLATE_SIZE/2*(-sin(-rstat->r) - cos(-rstat->r)) ) ),
	       CV_RGB(255,255,0), 1, CV_AA, 0 );
      
      // detect rect
/*      cvLine ( output_img,
	       cvPoint( (int)(rstat->x + TEMPLATE_SIZE + TEMPLATE_SIZE/2*(cos(-rstat->r) - sin(-rstat->r)) ),
			(int)(rstat->y + TEMPLATE_SIZE + TEMPLATE_SIZE/2*(sin(-rstat->r) + cos(-rstat->r)) ) ),
	       cvPoint( (int)(rstat->x + TEMPLATE_SIZE + TEMPLATE_SIZE/2*(-cos(-rstat->r) - sin(-rstat->r)) ),
			(int)(rstat->y + TEMPLATE_SIZE + TEMPLATE_SIZE/2*(-sin(-rstat->r) + cos(-rstat->r)) ) ),
	       CV_RGB(255,255,0), 1, CV_AA, 0 );
      cvLine ( output_img,
	       cvPoint( (int)(rstat->x + TEMPLATE_SIZE + TEMPLATE_SIZE/2*(cos(-rstat->r) - sin(-rstat->r)) ),
			(int)(rstat->y + TEMPLATE_SIZE + TEMPLATE_SIZE/2*(sin(-rstat->r) + cos(-rstat->r)) ) ),
	       cvPoint( (int)(rstat->x + TEMPLATE_SIZE + TEMPLATE_SIZE/2*(cos(-rstat->r) + sin(-rstat->r)) ),
			(int)(rstat->y + TEMPLATE_SIZE + TEMPLATE_SIZE/2*(sin(-rstat->r) - cos(-rstat->r)) ) ),
	       CV_RGB(255,255,0), 1, CV_AA, 0 );
      cvLine ( output_img,
	       cvPoint( (int)(rstat->x + TEMPLATE_SIZE + TEMPLATE_SIZE/2*(cos(-rstat->r) + sin(-rstat->r)) ),
 			(int)(rstat->y + TEMPLATE_SIZE + TEMPLATE_SIZE/2*(sin(-rstat->r) - cos(-rstat->r)) ) ),
	       cvPoint( (int)(rstat->x + TEMPLATE_SIZE + TEMPLATE_SIZE/2*(-cos(-rstat->r) + sin(-rstat->r)) ),
			(int)(rstat->y + TEMPLATE_SIZE + TEMPLATE_SIZE/2*(-sin(-rstat->r) - cos(-rstat->r)) ) ),
	       CV_RGB(255,255,0), 1, CV_AA, 0 );
      cvLine ( output_img,
	       cvPoint( (int)(rstat->x + TEMPLATE_SIZE + TEMPLATE_SIZE/2*(-cos(-rstat->r) - sin(-rstat->r)) ),
			(int)(rstat->y + TEMPLATE_SIZE + TEMPLATE_SIZE/2*(-sin(-rstat->r) + cos(-rstat->r)) ) ),
	       cvPoint( (int)(rstat->x + TEMPLATE_SIZE + TEMPLATE_SIZE/2*(-cos(-rstat->r) + sin(-rstat->r)) ),
			(int)(rstat->y + TEMPLATE_SIZE + TEMPLATE_SIZE/2*(-sin(-rstat->r) - cos(-rstat->r)) ) ),
	       CV_RGB(255,255,0), 1, CV_AA, 0 );*/
      
      // detect rect
/*      cvLine ( output_img,
	       cvPoint( (int)(rstat->x + TEMPLATE_SIZE/2 + TEMPLATE_SIZE/2*(cos(-rstat->r) - sin(-rstat->r)) ),
			(int)(rstat->y  + TEMPLATE_SIZE/2*(sin(-rstat->r) + cos(-rstat->r)) ) ),
	       cvPoint( (int)(rstat->x + TEMPLATE_SIZE/2 + TEMPLATE_SIZE/2*(-cos(-rstat->r) - sin(-rstat->r)) ),
			(int)(rstat->y  + TEMPLATE_SIZE/2*(-sin(-rstat->r) + cos(-rstat->r)) ) ),
	       CV_RGB(255,255,0), 1, CV_AA, 0 );
      cvLine ( output_img,
	       cvPoint( (int)(rstat->x + TEMPLATE_SIZE/2 + TEMPLATE_SIZE/2*(cos(-rstat->r) - sin(-rstat->r)) ),
			(int)(rstat->y  + TEMPLATE_SIZE/2*(sin(-rstat->r) + cos(-rstat->r)) ) ),
	       cvPoint( (int)(rstat->x + TEMPLATE_SIZE/2 + TEMPLATE_SIZE/2*(cos(-rstat->r) + sin(-rstat->r)) ),
			(int)(rstat->y  + TEMPLATE_SIZE/2*(sin(-rstat->r) - cos(-rstat->r)) ) ),
	       CV_RGB(255,255,0), 1, CV_AA, 0 );
      cvLine ( output_img,
	       cvPoint( (int)(rstat->x + TEMPLATE_SIZE/2 + TEMPLATE_SIZE/2*(cos(-rstat->r) + sin(-rstat->r)) ),
 			(int)(rstat->y  + TEMPLATE_SIZE/2*(sin(-rstat->r) - cos(-rstat->r)) ) ),
	       cvPoint( (int)(rstat->x + TEMPLATE_SIZE/2 + TEMPLATE_SIZE/2*(-cos(-rstat->r) + sin(-rstat->r)) ),
			(int)(rstat->y  + TEMPLATE_SIZE/2*(-sin(-rstat->r) - cos(-rstat->r)) ) ),
	       CV_RGB(255,255,0), 1, CV_AA, 0 );
      cvLine ( output_img,
	       cvPoint( (int)(rstat->x + TEMPLATE_SIZE/2 + TEMPLATE_SIZE/2*(-cos(-rstat->r) - sin(-rstat->r)) ),
			(int)(rstat->y  + TEMPLATE_SIZE/2*(-sin(-rstat->r) + cos(-rstat->r)) ) ),
	       cvPoint( (int)(rstat->x + TEMPLATE_SIZE/2 + TEMPLATE_SIZE/2*(-cos(-rstat->r) + sin(-rstat->r)) ),
			(int)(rstat->y  + TEMPLATE_SIZE/2*(-sin(-rstat->r) - cos(-rstat->r)) ) ),
	       CV_RGB(255,255,0), 1, CV_AA, 0 );*/
      


      //出力画像表示
      cvShowImage(MAP_WINDOW, output_img);
      //カメラのキャプチャ画像（共有メモリから取得）を表示
      cvShowImage("CameraWindow",capture_img);

#ifdef BIG_WINDOW
      //出力画像を引き伸ばして表示
      cvResize (output_img, big_img, CV_INTER_NN);
      cvShowImage(BIG_WINDOW, big_img);
#endif

//以下，画像をファイルに書き出し
#ifdef SAVE_TRACK
      sprintf( fname, SAVE_TRACK"map_%05d.png", count++ );
//      cvSaveImage( fname, output_img );
#endif
      //cvSaveImage( "test.png", output_img );

      //usleep(2000);

      key = cvWaitKey(50) & 0xff; //なぜか& 0xffがないとキー入力が効かない
      switch (key)
        {
        case 0x1b: // exit
          goto exit;
        case 'q':  // exit
          goto exit;
        case 'r':  // reset
	  map_img = cvLoadImage (MAP_SHOW, CV_LOAD_IMAGE_COLOR /* CV_LOAD_IMAGE_GRAYSCALE */);
	  printf("Map reset!");
          break;
        }
    }

 exit:;


#ifdef BIG_WINDOW
  cvDestroyWindow (BIG_WINDOW);
#endif
  cvDestroyWindow (MAP_WINDOW);

  return 0;
}

//出力画像上でのマウスクリックに対する動作の定義
static CvPoint lbuttondown_pt;

void on_mouse(int event, int x, int y, int flags, void* param)
{
  //左ボタンだけを使用
  //押したところが位置の中心
  //そこからドラッグしてリリース
  //中心点（押した点）とリリース点を結ぶ線分がロボットの方位角を示す
  switch (event)
    {
    case CV_EVENT_LBUTTONDOWN:
      lbuttondown_pt.x = x;
      lbuttondown_pt.y = y;
      rstat->x = x-TEMPLATE_SIZE/2;
      rstat->y = y-TEMPLATE_SIZE/2;
      break;
    case CV_EVENT_LBUTTONUP:
      rstat->r = atan2(x-lbuttondown_pt.x,y-lbuttondown_pt.y);
      printf("rstat->(x,y,r)=(%4d,%4d, %3.3f )\n", lbuttondown_pt.x-TEMPLATE_SIZE/2, lbuttondown_pt.y-TEMPLATE_SIZE/2, rstat->r );
      break;
    }
}
