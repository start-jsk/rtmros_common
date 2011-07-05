#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "PathPlanning.h"
#include "planner.h"
#include "PathType.h"

void path2pathseq(PathMapPtr a_path, IIS::TimedPath2DSeq *out_path);


/**/
PathMapPtr make_path(void){
  PathMapPtr a_path;

  a_path = (PathMapPtr)malloc(sizeof(PathMap));
  if(!a_path)return 0;

  a_path->first_node = 0;
  a_path->last_node = 0;
  a_path->node_num = 0;
  a_path->link_num = 0;

  a_path->start_node = 0;
  a_path->goal_node = 0;

  return a_path;
}

void delete_path(PathMapPtr a_path){
  /*delete all path information*/
  int i,n;
  WayPointPtr a_node,next_node;

  printf("delete all path data\n");
  a_node = a_path->first_node;
  n = a_path->node_num;
  for(i = 0; i < n; i++){
    next_node = a_node->next;
    delete_node(a_path, a_node);
    a_node = next_node;
  }
  printf("%d %d\n", a_path->node_num, a_path->link_num);
  
  free((char*)a_path);

  printf("done.\n");
}

/**/
void print_path_info(PathMapPtr a_path){
  printf("----- path information -----\n");
  printf("node num \t:%d\n",a_path->node_num);
  printf("link num \t:%d\n",a_path->link_num);

  if(a_path->start_node){
    printf("start %f %f (%f %f)\n",a_path->start_x, a_path->start_y,
	   a_path->start_node->x, a_path->start_node->y);
  }
  if(a_path->goal_node){
    printf("goal  %f %f (%f %f)\n",a_path->goal_x, a_path->goal_y,
	   a_path->goal_node->x, a_path->goal_node->y);
  }
}

/**/
WayPointPtr make_node(PathMapPtr a_path, double x, double y){
  WayPointPtr a_node;

  a_node = (WayPointPtr)malloc(sizeof(WayPoint));
  a_node->x = x;
  a_node->y = y;
  a_node->link_num = 0;
  a_node->flag =0;
  a_node->cost = 10000000000.0;
  a_node->from = 0;

  if(!a_path->first_node){
    a_path->first_node = a_node;
    a_path->last_node = a_node;
  }else{
    a_path->last_node->next = a_node;
    a_path->last_node = a_node;
  }
  a_node->next =0;

  a_path->node_num++;

  return a_node;
}

int delete_node(PathMapPtr a_path, WayPointPtr a_node){
  int i;

  if(!a_node)return 0;

  /*delete all links*/
  for(i = 0; i < a_node->link_num; i++)
    delete_link(a_path,a_node->links[i]);
  a_path->node_num--;

  printf("delete node\n");
  free((char*)a_node);
  return 1;
}
/**/

WayPointPtr nth_node(PathMapPtr a_path,int n){
  WayPointPtr a_node;
  int i;

  a_node = a_path->first_node;
  
  for(i =0;i < n;i++){
    if(a_node->next)a_node= a_node->next;
    else{
      printf("ss\n");
      return 0; 
    
    }
  }

  return a_node;
}
/**/
LinkPtr make_link(PathMapPtr a_path, WayPointPtr node1, WayPointPtr node2, double cost){
  LinkPtr a_link;

  if(node1->link_num > INT_SEC_NUM-1|| 
     node2->link_num > INT_SEC_NUM-1 )return 0;/*cannot add link*/

  a_link = (LinkPtr)malloc(sizeof(Link));
  if(!a_link)return 0;
  
  /*regist node information*/
  a_link->node1 = node1;
  a_link->node2 = node2;
  a_link->cost = cost;
  a_link->color = 0;
  a_link->length =0;
  a_link->width = 0;
  a_link->maxspeed = 0;

  if(cost <= 0)a_link->cost = sqrt((node1->x-node2->x)*(node1->x-node2->x)+
				   (node1->y-node2->y)*(node1->y-node2->y));


  /*regist new link pointer*/
  node1->links[node1->link_num++] = a_link;
  node2->links[node2->link_num++] = a_link;

  a_path->link_num++;

  return a_link;
}

int delete_link(PathMapPtr a_path, LinkPtr a_link){
  int i;

  if(!a_link)return -1;

  /*search linked pointer in table on node1*/
  for(i = 0;i < a_link->node1->link_num;i++)
    if(a_link->node1->links[i]==a_link)break;

  /*compress table*/
  if(i !=  a_link->node1->link_num){
    //printf("find \n");
    for(;i < a_link->node1->link_num-1;i++)
      a_link->node1->links[i] = a_link->node1->links[i+1];
    a_link->node1->link_num--;
  }else{
     printf("can not find \n");
  }

  /*search linked pointer in table on node2*/
  for(i = 0;i < a_link->node2->link_num;i++)
    if(a_link->node2->links[i]==a_link)break;

  /*compress table*/
  if(i !=  a_link->node2->link_num){
    //   printf("find \n");
    for(;i < a_link->node2->link_num-1;i++)
      a_link->node2->links[i] = a_link->node2->links[i+1];
    a_link->node2->link_num--;
  }else{
    printf("can not find \n");
  }

  a_path->link_num--;
  printf("delete link\n");
  free((char*)a_link);
 
  return 1;
}


/**/
int load_path_file(const char *file_name, PathMapPtr a_path){
  FILE *path_file;

  char kind;
  double x,y,theta,cost;
  WayPointPtr a_node;
  int n1,n2;
  int len;
  path_file = fopen(file_name,"r");
  if(!path_file){
    fprintf(stderr,"cannot open the assigned file.\n");
    return 0;
  }


  while(fscanf(path_file,"%c",&kind) != EOF){
    switch(kind){
    case 'n':
      len = fscanf(path_file,"%lf %lf %lf", &x, &y,&theta);    
      if(!(a_node = make_node(a_path, x, y)))
	printf("make_node error\n");
      printf("node (%f, %f)\n",x,y);
      break;
    case 'l':
      len = fscanf(path_file,"%d %d %lf", &n1,&n2,&cost);    
      if(!make_link(a_path, nth_node(a_path,n1), nth_node(a_path,n2), cost))
	printf("make_link error\n");
      printf("link %d-%d cost %f\n",n1,n2,cost);
      break;
    default:
      break;
    }
  } 
  return 1;
}

/*search nearest node (brute force search)*/
WayPointPtr search_nearest_node(PathMapPtr a_path, double x, double y){
  int i;
  WayPointPtr a_node,nearest_node;
  double d,min_d;

  a_node = a_path->first_node;
  min_d = 10000;
  printf("input %f %f\n",x,y);
  for(i = 0;i < a_path->node_num;i++){
    if(!a_node)return 0; 
   d = (x -a_node->x)*(x-a_node->x) + (y-a_node->y)*(y-a_node->y);
    if(min_d > d){
      min_d = d;
      nearest_node = a_node;
    }
    a_node = a_node->next;
 
  }
  printf("near  %f %f\n",nearest_node->x,nearest_node->y);
  return nearest_node;
}



LinkPtr search_nearest_link(PathMapPtr a_path, double x, double y){
  LinkPtr nearest_link=0;

  return nearest_link;

}

/*input current position*/
void set_start_position(PathMapPtr a_path, double x, double y){

  a_path->start_x = x;
  a_path->start_y = y;
  a_path->start_node = search_nearest_node(a_path,x,y);
  a_path->start_node->cost = 0;
}

/*input target  position*/
void set_goal_position(PathMapPtr a_path, double x, double y){
  a_path->goal_x = x;
  a_path->goal_y = y;
  a_path->goal_node = search_nearest_node(a_path,x,y);

}


WayPointPtr next_queue(QueueListHandle a_queue){
  QueueListPtr next_queue;
  WayPointPtr a_node;
  
    if(!*a_queue)return 0;

  a_node = (*a_queue)->node;

  next_queue= (*a_queue)->next;
  free((char*)(*a_queue));
  (*a_queue) = next_queue;

  return a_node;
}


/*a_queueの後ろにキューをつくる*/
QueueListPtr add_queue(QueueListHandle first_queue, WayPointPtr a_node){
  QueueListPtr new_queue;
  QueueListPtr a_queue;

  a_queue = *first_queue;

  new_queue = (QueueListPtr)malloc(sizeof(QueueList));

  if(!a_queue){
    printf("add new\n");  
    *first_queue = new_queue;
    new_queue->next = 0;
    new_queue->node = a_node;
  }else if(a_queue->node->cost < a_node->cost){
    printf("add first\n");
    *first_queue = new_queue;
    new_queue->next = a_queue;
    new_queue->node = a_node;
  }else{
    while(a_queue->next){
      if(a_queue->next->node->cost > a_node->cost){
	printf("add \n");
	new_queue->next = a_queue->next;
	a_queue->next   = new_queue;
	new_queue->node = a_node;
	break;
      }
      a_queue = a_queue->next;
    }
  }

  return new_queue;
}



/*solve*/
PathMapPtr solve_minimum_path(PathMapPtr a_path){
  QueueListPtr queue_list;
  int i,goal_reached;
  double min_cost;
  WayPointPtr a_node,new_node,bef_node;
  PathMapPtr min_path;
 
  min_cost = 1e100; 
  queue_list = 0;
  add_queue(&queue_list, a_path->start_node);
  printf("start %f %f\n",a_path->start_node->x,a_path->start_node->y);
  printf("goal  %f %f\n",a_path->goal_node->x,a_path->goal_node->y);

  min_path = make_path();
  goal_reached = 0;
  while((a_node=next_queue(&queue_list)) != 0 && !goal_reached){/*キューから一つ読み出す*/

    if(a_node == a_path->goal_node){
      printf("goal\n");
      if(min_cost > a_node->cost)min_cost = a_node->cost;
    }

    if(a_node->cost > min_cost)continue;

    printf("%f %f %f \n",a_node->x, a_node->y,a_node->cost);
    /*ノードをキューに登録する*/
    for(i = 0;i < a_node->link_num; i++){  
      if(a_node->links[i]->node1 == a_node){
	
	if(a_node->cost + a_node->links[i]->cost <
	   a_node->links[i]->node2->cost) {
	  add_queue(&queue_list, a_node->links[i]->node2);	  
	  a_node->links[i]->node2->from = a_node;
	  a_node->links[i]->color = 1;
	  a_node->links[i]->node2->cost = 
	    a_node->cost + a_node->links[i]->cost;

	}
      }else{
	if(a_node->cost + a_node->links[i]->cost <
	   a_node->links[i]->node1->cost){
	  add_queue(&queue_list, a_node->links[i]->node1);
	  a_node->links[i]->color = 1;
	  a_node->links[i]->node1->from = a_node;
	  a_node->links[i]->node1->cost = 
	    a_node->cost + a_node->links[i]->cost;
	  
	  //	  if(a_node->cost > min_cost){goal_reached = 1;break;}
	}
      }
    }
  }

  /*back trace*/
  a_node = a_path->goal_node;
  bef_node = make_node(min_path, a_node->x, a_node->y);
  printf("%f %f\n",a_node->x,a_node->y );
  a_node = a_node->from;
  while(a_node){
    printf("%f %f\n",a_node->x,a_node->y );
    new_node = make_node(min_path,a_node->x,a_node->y);
    make_link(min_path,new_node,bef_node,1);
    a_node = a_node->from;

    bef_node =new_node;

  }

  //clear node cost
  a_node = a_path->first_node;
  for(i = 0;i < a_path->node_num;i++){
    if(!a_node)return 0; 
    a_node ->cost = 1e100;
    a_node->flag =0;
    a_node->from = 0;
    a_node = a_node->next;
  }



  return min_path;
}

void write_path(FILE *a_file, PathMapPtr a_path){
  WayPointPtr a_node;
  int i;

  a_node = a_path->first_node;
  while(a_node){
    for(i =0;i <  a_node->link_num;i++){
      fprintf(a_file, "%f %f \n%f %f\n\n",
	      a_node->links[i]->node1->x,a_node->links[i]->node1->y,
	      a_node->links[i]->node2->x,a_node->links[i]->node2->y);
    }
    a_node = a_node->next;
  }


}

void path2pathseq(PathMapPtr a_path, IIS::TimedPath2DSeq *out_path){
  WayPointPtr a_node;
  int i,num;

  a_node = a_path->first_node;
    if(!a_node)return;
  
  i=1;
  while(a_node->next){
    i++;
    a_node = a_node->next;
  }
  num = i;
  //  if(i<=1)return;

  //  out_path->path_list.length(i);
  out_path->id.length(i);
  out_path->pose.length(i);
  out_path->velocity.length(i);
  out_path->error.length(i);

  // printf("1 %d\n",i);  
  
  a_node = a_path->first_node;
  /*//i = 0;
  out_path->path_list[i].type = RUN_LINEFOLLOW;
  out_path->path_list[i].v = 0.3;
  out_path->path_list[i].x = a_node->x;
  out_path->path_list[i].y = a_node->y;
  out_path->path_list[i].theta =
    atan2(a_node->y-a_node->next->y,
	  a_node->x-a_node->next->x );
  */
  /*旧
    out_path->path_list[i].type = RUN_LINEFOLLOW;
    out_path->path_list[i].v = 0.3;
    out_path->path_list[i].x = a_node->x;
    out_path->path_list[i].y = a_node->y;
  */
   
  while(a_node->next){ printf("%d\n",i);
    i--;
    out_path->id[i] = i;
    out_path->velocity[i].vx = 1.0;
    out_path->velocity[i].vy = 0.0;
    out_path->velocity[i].va = 0.0;
    out_path->pose[i].position.x = a_node->x;
    out_path->pose[i].position.y = a_node->y;
    out_path->error[i] = 0.0;
    out_path->pose[i].heading =
      atan2(a_node->y-a_node->next->y,
	    a_node->x-a_node->next->x );
    
    a_node = a_node->next;
    //i++;
  }

  i--;
  out_path->id[0] = 0;
  out_path->velocity[0].vx = 1.0;
  out_path->velocity[0].vy = 0.0;
  out_path->velocity[0].va = 0.0;
  out_path->pose[0].position.x = a_node->x;
  out_path->pose[0].position.y = a_node->y;
  out_path->pose[0].heading = 0;
  out_path->error[0] = 0.0;


  //  printf("2\n");  
  //out_path->path_list[0].theta = 0;//out_path->path_list[i-1].theta ;
  //    a_node->y-a_node->links[0]->node2->y,
  //  a_node->x-a_node->links[0]->node2->x);

  //  printf("3\n");  
}

void draw_path(PathMapPtr a_path,PathMapPtr min_path){
  static FILE *gnuplot;
  FILE *path_file, *path_file2;
  //  WayPointPtr a_node;
 
  if(!gnuplot){
    gnuplot =popen("/usr/bin/gnuplot","w");
    fprintf(gnuplot,"set term x11\n");
  }
  path_file = fopen("path","w");
  path_file2 = fopen("path2","w");

  write_path(path_file,a_path);
  write_path(path_file2,min_path);
  fclose(path_file);
  fclose(path_file2);

  fprintf(gnuplot,"set xrange [-16:16]\n");
  fprintf(gnuplot,"set yrange [-12:12]\n");
  fprintf(gnuplot,"plot 'path' w l, 'path2' w l\n");
  //  fprintf(gnuplot,"plot 'path' w l\n");
  fflush(gnuplot);
  //  getchar();
  //pclose(gnuplot);

}

#if 0
/**/
int main(int argc, char* argv[]){
  PathMapPtr minimum_path;
  PathMapPtr all_path;
  
  /*load path file*/
  all_path = make_path();
  load_path_file(argv[1], all_path);

  /*input current position*/
  set_start_position(all_path, 5, 3);

  /*input target  position*/
  set_goal_position(all_path, 1, 1);
  
  /*print created path information*/
  print_path_info(all_path);

  /*solve*/
  minimum_path=solve_minimum_path(all_path);
  draw_path(all_path,minimum_path);
  

  //  delete_path(all_path);
}
#endif 
