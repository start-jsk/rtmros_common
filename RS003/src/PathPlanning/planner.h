#ifndef __PLANNER__
#define __PLANNER__

//#include "PathPlanning.h"

#define INT_SEC_NUM 8


typedef struct TD_node *WayPointPtr; 
typedef struct TD_link *LinkPtr; 
typedef struct TD_path *PathMapPtr; 

typedef struct TD_path{
  WayPointPtr first_node;
  WayPointPtr last_node;

  double start_x;
  double start_y;
  WayPointPtr start_node;

  double goal_x;
  double goal_y;
  WayPointPtr goal_node;

  int node_num;
  int link_num;
}PathMap;

typedef struct TD_node{
  double x;
  double y;
  int    flag;
  double cost;
  
  int     link_num;
  LinkPtr links[INT_SEC_NUM];
  WayPointPtr from;
  WayPointPtr next;
}WayPoint;

typedef struct TD_link{
  double cost;
  double length;
  double color;
  double width;
  double maxspeed;

  WayPointPtr node1;
  WayPointPtr node2;
}Link;

typedef struct TD_queue_list *QueueListPtr,**QueueListHandle;
typedef struct TD_queue_list{
  WayPointPtr node;
  QueueListPtr next;
}QueueList;

/**/
PathMapPtr make_path(void);
void delete_path(PathMapPtr a_path);
/**/
void print_path_info(PathMapPtr a_path);
/**/
WayPointPtr make_node(PathMapPtr a_path, double x, double y);
int delete_node(PathMapPtr a_path, WayPointPtr a_node);
WayPointPtr nth_node(PathMapPtr a_path,int n);
/**/
LinkPtr make_link(PathMapPtr a_path, WayPointPtr node1, WayPointPtr node2, double cost);
int delete_link(PathMapPtr a_path, LinkPtr a_link);
/**/
int load_path_file(const char *file_name, PathMapPtr a_path);
/*search nearest node (brute force search)*/
WayPointPtr search_nearest_node(PathMapPtr a_path, double x, double y);
LinkPtr search_nearest_link(PathMapPtr a_path, double x, double y);
/*input current position*/
void set_start_position(PathMapPtr a_path, double x, double y);
/*input target  position*/
void set_goal_position(PathMapPtr a_path, double x, double y);
WayPointPtr next_queue(QueueListHandle a_queue);
/*a_queueの後ろにキューをつくる*/
QueueListPtr add_queue(QueueListHandle first_queue, WayPointPtr a_node);
/*solve*/
PathMapPtr solve_minimum_path(PathMapPtr a_path);
void write_path(FILE *a_file, PathMapPtr a_path);
//void path2pathseq(PathMapPtr a_path, PathSeq *out_path);
void draw_path(PathMapPtr a_path,PathMapPtr min_path);
#endif 
