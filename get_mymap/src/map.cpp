#include <ros/ros.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include "std_msgs/Header.h"

//с целью добавить нахождение позиции робота
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/observation_buffer.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>




#include <message_filters/subscriber.h>

#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <costmap_2d/cost_values.h>



#include <cstdio>
#include <string>
#include <algorithm>
#include <vector>


#include <cv_bridge/cv_bridge.h>
#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
//#include <opencv/cv.hpp>

//чтобы работал мув бэйз
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


#define DEBUG

using namespace cv;
using namespace std;
using namespace costmap_2d;

//ВНИМАНИЕ
//Функция at работает иначе, x это y, y это x.
//За счет этого карта в opencv будет восприниматься с такими координатами. То есть не (x,y) а (x,y)



//объявляем чтобв работал move_base
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//Флаг, если найдена новая цель то флаг поднимается
int new_goal_detected = 0;
float goal_x = 0;
float goal_y = 0;





long long int Nwhite = 0; //число белых исследованных клеток на поле

//Mat img;
Mat map_cv(Size(1000,1000), CV_8U, Scalar::all(0));
Mat map_show(Size(400,400), CV_8U, Scalar::all(0));
//Mat black2(Size(400,400), CV_8U, Scalar::all(0));

//Позиция робота относительно центра карты
float Robot_pose_from_center_X = 0.0;
float Robot_pose_from_center_Y = 0.0;

//Позиция робота на карте относительно ее правого угла в пикселях
int Robot_map_pose_X = 0;
int Robot_map_pose_Y = 0;


void get_robot_pose_in_pixels (const nav_msgs::MapMetaData& info){
  //Находим координаты правого нижнего угла карты относительно ее центра
  float map_corner_X = info.origin.position.x;
  float map_corner_Y = info.origin.position.y;
  //Находим положение робота относительно правого нижнего угла карты в пикселях
  Robot_map_pose_X = int(20.0*(Robot_pose_from_center_X - map_corner_X));
  Robot_map_pose_Y = int(20.0*(Robot_pose_from_center_Y - map_corner_Y));
  ROS_INFO("Robot on map:  X = %d ,  Y = %d", Robot_map_pose_X, Robot_map_pose_Y);
    
}

void fill_opencv_map (const nav_msgs::OccupancyGrid::ConstPtr& msg, const std_msgs::Header header, const nav_msgs::MapMetaData info){
    for (unsigned int y = 0; y < info.height; y++){
    for (unsigned int x = 0; x < info.width; x++){
      
      if(msg->data[x+ info.width* y]==-1){ // -1 is unknown
      
        map_cv.at<unsigned char>(x, y) = 150;

      } else if(msg->data[x+ info.width * y]==0) { // свободно
        map_cv.at<unsigned char>(x, y) = 255;
      }
    }
  }
}

void make_walls_bigger(const nav_msgs::OccupancyGrid::ConstPtr& msg, const std_msgs::Header header, const nav_msgs::MapMetaData info){
    //проходим еще раз, чтобы выставить радиусы стенам.
  for (unsigned int y = 0; y < info.height; y++){
    for (unsigned int x = 0; x < info.width; x++){
      
      
      if(msg->data[x+ info.width * y]==100){ // если занято, то наносим круг
        
        circle(map_cv, cv::Point(y, x), 2, Scalar::all(0), -1);
        

      } 
    }
  }
}

//волновой алгоритм (поиск в ширину)
Point find_closest_unknown_cell(void){
  Point target;
  //В эту точку запишем направление цели от найденной неизвестной
  Point direction;

  //создаем массив в который будем записывать числа (вектор на n строк, m элементов)
  int n = 1000;
  int m = 1000;
  vector < vector <int> > cost_of_path_map(n, vector <int> (m) );

  //запишем в этот массив стены, неизвестные точки
  for (int y = 0; y < m; y++){
    for (int x = 0; x < n; x++){
      if (map_cv.at<unsigned char>(x, y) == 150){
      cost_of_path_map[x][y] = -999999999; //неизвестная клетка
      } else if (map_cv.at<unsigned char>(x, y) == 0){
        cost_of_path_map[x][y] = -2000000000; //стена
      } else if (map_cv.at<unsigned char>(x, y) == 255){
        cost_of_path_map[x][y] = -1; //свободная клетка
      }
      //ROS_INFO("Map_of_wave[%d][%d] = %d",x,y,cost_of_path_map[x][y]);
    }
  }

  //создаем волну 1 и волну 2. По очереди в них будем записывать координаты клеток наших волн
  vector <Point> wave_1;
  vector <Point> wave_2;
  //смешение относительно текущей точки
  const int dx[] = {0,1,0,-1};
  const int dy[] = {-1,0,1,0};
  int nstep = 0;
  Point start(Robot_map_pose_X, Robot_map_pose_Y);
  cost_of_path_map[start.x][start.y] = 0;
  wave_1.push_back(Point(start.x,start.y));

  while(1){
    
    wave_2.clear();
    nstep++;
    //расширяем в 4 направления точки из первой волны. Новые записываем во вторую
    //ROS_INFO("Wave_1 size is %ld",wave_1.size());
    for (int i = 0; i < wave_1.size(); i++){
      for (int side = 0; side < 4; side++){
        //если это свободная клетка
        if(cost_of_path_map[wave_1[i].x + dx[side]][wave_1[i].y + dy[side]] == -1){
          cost_of_path_map[wave_1[i].x + dx[side]][wave_1[i].y + dy[side]] = nstep;
          wave_2.push_back(Point(wave_1[i].x + dx[side], wave_1[i].y + dy[side]));
          //если это неизвестная клетка
          //ROS_INFO("Map cell is  %d", cost_of_path_map[wave_1[i].x + dx[side]][wave_1[i].y + dy[side]]);
        } else if (cost_of_path_map[wave_1[i].x + dx[side]][wave_1[i].y + dy[side]] == -999999999){
          //записываем координаты неизвестной точки
          target.x = wave_1[i].x + dx[side];
          target.y = wave_1[i].y + dy[side];
          //записываем направление в неизвестность от неизвестной точки
          //direction.x = wave_1[i].x + 10*dx[side];
          //direction.y = wave_1[i].y + 10*dy[side];
          //выходим из цикла
          goto way_search;

        }
      }
    }
    wave_1.clear();
    nstep++;
    //ROS_INFO("Wave_2 size is %ld",wave_2.size());
    //расширяем в 4 направления точки из второй волны. Новые записываем в первую
    for (int i = 0; i < wave_2.size(); i++){
      for (int side = 0; side < 4; side++){
        //если это свободная клетка
        if(cost_of_path_map[wave_2[i].x + dx[side]][wave_2[i].y + dy[side]] == -1){
          cost_of_path_map[wave_2[i].x + dx[side]][wave_2[i].y + dy[side]] = nstep;
          wave_1.push_back(Point(wave_2[i].x + dx[side], wave_2[i].y + dy[side]));
          //если это неизвестная клетка
        }else if (cost_of_path_map[wave_2[i].x + dx[side]][wave_2[i].y + dy[side]] == -999999999){
          //записываем координаты неизвестной точки
          target.x = wave_2[i].x + dx[side];
          target.y = wave_2[i].y + dy[side];
          //direction.x = wave_2[i].x + 10*dx[side];
          //direction.y = wave_2[i].y + 10*dy[side];
          //выходим из цикла
          goto way_search;
        }
      }
    }
  }
    //вышли из цикла. Возвращаем неизвестную точку
    way_search:

    //теперь сделаем площадь по неизвестной области
    //======================================================================

  cost_of_path_map[target.x][target.y] = 0;


  nstep = 0;
  wave_1.clear();
  wave_1.push_back(Point(target.x,target.y));

  //сохраняем последнюю клетку если мы не достигнем 20
  Point last_cell(target.x,target.y);

  while(1){
    
    wave_2.clear();
    nstep++;
    //расширяем в 4 направления точки из первой волны. Новые записываем во вторую
    //ROS_INFO("Wave_1 size is %ld",wave_1.size());
    for (int i = 0; i < wave_1.size(); i++){
      for (int side = 0; side < 4; side++){
        
        if(cost_of_path_map[wave_1[i].x + dx[side]][wave_1[i].y + dy[side]] == -999999999){
          cost_of_path_map[wave_1[i].x + dx[side]][wave_1[i].y + dy[side]] = nstep;
          wave_2.push_back(Point(wave_1[i].x + dx[side], wave_1[i].y + dy[side]));
          last_cell.x = wave_1[i].x + dx[side];
          last_cell.y = wave_1[i].y + dy[side];
          
        }
      }
    }
    wave_1.clear();
    nstep++;
    //ROS_INFO("Wave_2 size is %ld",wave_2.size());
    //расширяем в 4 направления точки из второй волны. Новые записываем в первую
    for (int i = 0; i < wave_2.size(); i++){
      for (int side = 0; side < 4; side++){
        
        if(cost_of_path_map[wave_2[i].x + dx[side]][wave_2[i].y + dy[side]] == -999999999){
          cost_of_path_map[wave_2[i].x + dx[side]][wave_2[i].y + dy[side]] = nstep;
          wave_1.push_back(Point(wave_2[i].x + dx[side], wave_2[i].y + dy[side]));
          //если это неизвестная клетка
          last_cell.x = wave_1[i].x + dx[side];
          last_cell.y = wave_1[i].y + dy[side];
        }
      }
    }
    //когда 20 волна, то сохраням точку из середины массива
    if(nstep == 20){
      ROS_INFO("We got 20 steps");
      if (wave_1.size()){
        ROS_INFO("Wave 1 is big enough");
        direction.x = wave_1[int(wave_1.size()/2)].x;
        direction.y = wave_1[int(wave_1.size()/2)].y;
      goto draw_target;
      } else{
        ROS_INFO("Using last cell");
        direction.x = last_cell.x;
        direction.y = last_cell.y;
        goto draw_target;
      }
    }
  }

  draw_target:

  target.x = direction.x;
  target.y = direction.y;

  //нарисуем плюсик на цели
  map_cv.at<unsigned char>(target.x, target.y) = 50;
  map_cv.at<unsigned char>(target.x +1 , target.y) = 50;
  map_cv.at<unsigned char>(target.x - 1, target.y) = 50;
  map_cv.at<unsigned char>(target.x, target.y + 1) = 50;
  map_cv.at<unsigned char>(target.x, target.y - 1) = 50;


  return target;
}

void convert_map_coord_to_real(const nav_msgs::MapMetaData& info, Point target){
  float map_corner_X = info.origin.position.x;
  float map_corner_Y = info.origin.position.y;
  float target_x = (float)(target.x);
  float target_y = (float)(target.y);
  goal_x = target_x/20.0 + map_corner_X;
  goal_y = target_y/20.0 + map_corner_Y;
  ROS_INFO("SET TARGET (%f, %f)",goal_x,goal_y);
  new_goal_detected = 1;
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  ROS_DEBUG("sefsefsefsefsefse");
  std_msgs::Header header = msg->header;
  nav_msgs::MapMetaData info = msg->info;
  ROS_INFO("Got map %d %d", info.width, info.height);

  //получаем позицию робота в пикселях начиная с правого нижнего угла карты
  get_robot_pose_in_pixels(info); 

  //Перевод в формат opencv
  fill_opencv_map(msg, header, info); 
  
  //увеличиваем стены, чтобы затмить пробелы и получить радиус робота вдоль стены
  //без этой функции стены не отмечаются в принципе
  make_walls_bigger(msg, header, info); 

  //находим ближайшую неизветную точку, отмечаем ее плюсом и строим к ней путь
  Point target = find_closest_unknown_cell();

  convert_map_coord_to_real(info, target);
  //goal_x = target.x;
  //goal_y = target.y;
  //new_goal_detected = 1;

  map_cv.at<unsigned char>(Robot_map_pose_X, Robot_map_pose_Y) = 50;

  imshow("img", map_cv );
  waitKey(600);
}




int main(int argc, char **argv){

  
  
  ros::init(argc, argv, "map_node");
  ros::NodeHandle n;

  MoveBaseClient ac("tb3_1/move_base", true); //для работы мув бэйз
	
	while(!ac.waitForServer(ros::Duration(5.0))){ //ждем мув бэйз
		ROS_INFO("Waiting for move_base");
	}
  
  //Подписываемся на карту
  ros::Subscriber map_sub = n.subscribe("map",2000,mapCallback);

  tf::TransformListener listener; //для извлечения позиции робота на карте

  move_base_msgs::MoveBaseGoal goal;// цель для мувбэйз
  goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

  while (n.ok()){ 
    ros::spinOnce();
    tf::StampedTransform transform;

    try //Пробуем получить позицию робота относительно центра карты
      {
        listener.waitForTransform("/map", "/base_link", ros::Time::now(), ros::Duration(1.0));

        listener.lookupTransform("/map","/base_link", ros::Time(0), transform);

        //ROS_INFO("Got a transform! x = %f, y = %f",transform.getOrigin().x(),transform.getOrigin().y());
        //это дает позицию робота на карте
        Robot_pose_from_center_X = transform.getOrigin().x();
        Robot_pose_from_center_Y = transform.getOrigin().y();
        
      }
    catch (tf::TransformException ex)
      {
        ROS_ERROR("Nope! %s", ex.what());
      }

      //если появилась новая цель
      if(new_goal_detected){
        

        goal.target_pose.pose.position.x = goal_x;
		    goal.target_pose.pose.position.y = goal_y;
		    goal.target_pose.pose.orientation.w = 1.57;
        new_goal_detected = 0;

        ROS_INFO("Sending goal");
		    ac.sendGoal(goal);

        /*
        ac.waitForResult();
        
		    ros::Duration(1.0).sleep();
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		      ROS_INFO("target succeeded");
	      } else {
		      ROS_INFO("target failed");
	      }
        */
      }
  }


  ros::spin();
  return 0;
}





