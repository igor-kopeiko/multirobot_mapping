//Для получения карты
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
#include <queue>
#include <algorithm>

//работа опенсв
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

//чтобы работал мув бэйз
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


// ============================================================================================
// НАСТРОЙКИ АЛГОРИТМА ========================================================================

//Карта имеет смещение, поэтому вводим смещение в алгоритм, чтобы его компенсировать. ВЕЛИЧИНУ НЕ МЕНЯТЬ!!!
#define COORD_OFFSET 0.0 //0.385

//Смещение координат для локальной карты
#define COORD_OFFSET_LOCAL_MAP 0.385

//Количество роботов. При изменении необходимо выполнить добавить/убрать строки в функции main
#define ROBOT_AMOUNT 3

//На сколько шагов волна от границы может уходить в неизвестную область
//Половина этого числа будет определять насколько далеко цель будет находиться в неизвестности 
//При большом числе также может поглощадь больше соседних границ
//Необходимый размер волны в неизвестной области, чтобы ее внесли в список целей
//СТАВИТЬ ЧЕТНОЕ ЧИСЛО, по умолчанию 76
#define TARGET_DEEP 76

//Отклонения от лучшей комбинации с целью поиска схожих комбинаций. 
#define DEVIATION_SIZE 0.07


//делать ли калиброку для роботов (первый робот будет объезжать остальных для синхронизации карты)
#define CALIBRATE_ROBOTS false

//отдаляться от робота за которым следуем, если он слишком близко
#define GET_AWAY_FROM_FOLLOW_ROBOT false 


// КОНЕЦ НАСТРОЕК==============================================================================
// ============================================================================================

//значения клеток на карте OPENCV
#define CV_OCCUPED 0
#define CV_FREE 255
#define CV_UNKNOWN 150

//значения клеток на картах, которые создают и используют функции
#define MYCOST_OCCUPED -999
#define MYCOST_FREE -1
#define MYCOST_UNKNOWN -50
#define MYCOST_TARGET -10
#define MYCOST_ROBOT -30



using namespace cv;
using namespace std;
using namespace costmap_2d;

//публикация модицифированной карты
ros::Publisher modif_merged_map_publisher;


class MyRobot {
public:
  
  string name;

  //Позиция робота относительно центра карты
  Point2f pose_from_center;
  
  //Позиция робота на карте относительно ее правого угла в пикселях
  Point pose_on_map_in_pix;

  //путь до каждой цели
  vector <int> way_to_targets;
  //координаты каждой цели
  vector <Point> targets_coord;

  //флаг если находится в неизвестном пространестве, то ему нужна своя цель, примет значение true
  bool need_personal_target;
  //Флаг, если найдена новая цель то флаг станет true
  bool goal_detected;
  //Координаты цели робота
  Point2f goal;

  //координаты цели для отрисоки на карте
  Point goal_on_map;

  //Вокруг робота мы делаем свободную зону на карте opencv, чтобы его товарищи не воспринимали его стеной
  Point safe_zone[9];


  //можно ли следовать за этим роботом
  bool robot_can_have_folowers; 

  //количество роботов, которые следуют за этим роботом
  int followers_amount;
  
  //позиция в очереди за роботом, за которым мы следуем
  int pose_in_follow_queue;

  //Если робот долго на одном месте, то он признается застрявшим. Ему назначается персональная цель
  bool stuck = false;
  //Проверка что робот двигается
  int stuck_check = 0; //если значение достигнет 9, то робот застрял
  //возможная позиция, рядом с которой застрял робот
  Point possible_stuck_pose{Point(0,0)};

  //Методы класса
  void clear_ways_and_targets_coord(){
    way_to_targets.clear();
    targets_coord.clear();
  }
};


//Массив роботов, в нем хранится информация о всех роботах во время работы
MyRobot myrobots[ROBOT_AMOUNT];

//Перечень всех глобальных общих целей
//будем вносить сюда все достаточно большие цели
vector<Point> all_targets;


//ВНИМАНИЕ
//При разработке в функцию at ЗАПРЕЩАЕТСЯ передавать Point целиком.
//Нельзя at<unsigned char>(point)
//Нужно  at<unsigned char>(point.x, point,y)
//Функция at работает иначе чем point, x это y, y это x. В первую переменную пишем строку, во вторую столбец, поэтому ТАК!




//объявляем клиент для работы move_base
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


//Карта openCV для работы алгоритм.
Mat map_cv(Size(500,500), CV_8U, Scalar::all(0));

Mat map_cv0(Size(500,500), CV_8U, Scalar::all(0));
Mat map_cv1(Size(500,500), CV_8U, Scalar::all(0));
Mat map_cv2(Size(500,500), CV_8U, Scalar::all(0));

//карта, в увеличенном формате, которая будет выводиться пользователю
Mat map_show_zoom;

//размеры полученной карты
int map_width = 0;
int map_height = 0;


//Point map_center_in_pix;

//глубина цели в неизвестное пространство
int global_target_deep = TARGET_DEEP;


bool robot_use_local_map[3] = {true, true, true};


//функция находит положение каждого работа на карте и записывает в главный массив роботов
void get_robot_pose_in_pixels (const nav_msgs::MapMetaData& info){
  //Находим координаты правого нижнего угла карты относительно ее центра
  float map_corner_X = info.origin.position.x - COORD_OFFSET;
  float map_corner_Y = info.origin.position.y - COORD_OFFSET;
  //ROS_INFO("Map corner coord X = %f  Y = %f", map_corner_X, map_corner_Y);

  //Находим положение робота относительно правого нижнего угла карты в пикселях
  for (int r = 0; r < ROBOT_AMOUNT; r++){
  myrobots[r].pose_on_map_in_pix.x = int(20.0*(myrobots[r].pose_from_center.x - map_corner_X));
  myrobots[r].pose_on_map_in_pix.y = int(20.0*(myrobots[r].pose_from_center.y - map_corner_Y));
  }

  //запишем центр карты в пикселях
  //map_center_in_pix.x = int(20*(-map_corner_X));
  //map_center_in_pix.y = int(20*(-map_corner_Y));
}

//функция копирует известные и неизвестные ячейки из карты полученной карты в opencv карту
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

//функция копирует занятые ячейки с карты в карту opencv и увеличивает их
void make_walls_bigger(const nav_msgs::OccupancyGrid::ConstPtr& msg, const std_msgs::Header header, const nav_msgs::MapMetaData info){
    //проходим еще раз, чтобы выставить радиусы стенам.
  bool robot_is_detected_as_wall = false;
  for (unsigned int y = 0; y < info.height; y++){
    for (unsigned int x = 0; x < info.width; x++){
      if(msg->data[x+ info.width * y]==100){ // если занято, то наносим круг

        for (int r = 0; r < ROBOT_AMOUNT; r++){
          for (int i = 0; i < 9; i++){
            //если координаты совпадают хоть раз с сейф зоной, то стена не наносится
            if ((x == myrobots[r].safe_zone[i].x) && (y == myrobots[r].safe_zone[i].y)){
              robot_is_detected_as_wall = true;
            }
          }
        }
        //наносим круг в случае если робот не перекрывается стеной
        if (!robot_is_detected_as_wall){
          circle(map_cv, cv::Point(y, x), 2, Scalar::all(0), -1);
        } 
        //возвращем в исходное состояние
        robot_is_detected_as_wall = false;
      } 
    }
  }

  //нарисуем сейф зону вокруг каждого робота
  for (int r = 0; r < ROBOT_AMOUNT; r++){
    for (int i = 0; i < 9; i++){
      map_cv.at<unsigned char>(myrobots[r].safe_zone[i].x,myrobots[r].safe_zone[i].y) = 255;
    }
  }

}

//функция переводит координаты из пикселей в метры
void convert_goal_coord_from_map_pix_to_real(const nav_msgs::MapMetaData& info, int robot_index){
  float map_corner_X = info.origin.position.x - COORD_OFFSET;
  float map_corner_Y = info.origin.position.y - COORD_OFFSET;
  float target_x = (float)(myrobots[robot_index].goal_on_map.x);
  float target_y = (float)(myrobots[robot_index].goal_on_map.y);
  myrobots[robot_index].goal.x = target_x/20.0 + map_corner_X;
  myrobots[robot_index].goal.y = target_y/20.0 + map_corner_Y;
  //ROS_INFO("SET TARGET (%f, %f)",goal_x,goal_y);
  myrobots[robot_index].goal_detected = true;
  return;
}





// заполняем сейф зону роботов, зная его координаты
//9 ячеек (под роботом и рядом с ним) будут заполнены как свободные.
void set_safe_zone_for_robots(){

  const int dx[] = {0, 0, 1, 0, -1, 1, -1, 1, -1};
  const int dy[] = {0, -1, 0, 1, 0, 1, -1, -1, 1};

  for (int r = 0; r < ROBOT_AMOUNT; r++){
    //очищаем массив
    for (int i = 0; i < 9; i++){
      myrobots[r].safe_zone[i].x = 0;
      myrobots[r].safe_zone[i].y = 0;
    }

    //заполняем массив
    for (int i = 0; i < 9; i++){
      myrobots[r].safe_zone[i].x = myrobots[r].pose_on_map_in_pix.x + dx[i];
      myrobots[r].safe_zone[i].y = myrobots[r].pose_on_map_in_pix.y + dy[i];
    }
  }
}








//создать стандартную карту из карты opencv, но в формате int
vector < vector <int> > create_int_map(){
  int x_size = map_width;
  int y_size = map_height;
  vector < vector <int> > my_cost_map(x_size, vector <int> (y_size) );
  for (int y = 0; y < y_size; y++){
    for (int x = 0; x < x_size; x++){
      if (map_cv.at<unsigned char>(x, y) == CV_UNKNOWN){
        my_cost_map[x][y] = MYCOST_UNKNOWN; //неизвестная клетка
      } else if (map_cv.at<unsigned char>(x, y) == CV_OCCUPED){
        my_cost_map[x][y] = MYCOST_OCCUPED; //стена
      } else if (map_cv.at<unsigned char>(x, y) == CV_FREE){
        my_cost_map[x][y] = MYCOST_FREE; //свободная клетка
      }
    }
  }
  return my_cost_map;
}




vector<Point> find_any_free_cells(){ //перебираем все точки на карте и находим все свободные, при этом 3/4 всех точек пропускаем
  vector<Point> free_cells;
  for (int x = 0; x < map_cv.rows; x = x + 10){
    for (int y = 0; y < map_cv.cols; y = y + 10){
      if (map_cv.at<unsigned char>(x,y) == CV_FREE){
        free_cells.push_back(Point(x, y));
      }
    }
  }
  return free_cells;
}

//функция находит все границы от известной области. Известная область должна составлять более 50% от известных клеток на все карте
vector<Point> find_borders(){
  vector<Point> free_cells = find_any_free_cells();
  //ROS_WARN("FREE CELLS %ld", free_cells.size());


  //лучшие границы это те, где на максимум задействуются различные карты роботов
  vector<Point> best_borders;
  //лучшая комбинация где больше всего роботов, если роботов 2 это уже повод для перевода на глобальную карту,
  bool best_robots_use_global_map[ROBOT_AMOUNT] = {false, false, false};
  int best_global_users_amount = 0;
  // а если их 3, тогда все переходят на глобальную карту

      //считаем число клеток на локальных картах
    int total_free_cells_on_maps[3] = {0, 0, 0};
    for (int x = 0; x < map_width; x++){
      for (int y = 0; y < map_height; y++){
        if(map_cv0.at<unsigned char>(x, y) == CV_FREE){
          total_free_cells_on_maps[0]++;
        }
      }
    }
    for (int x = 0; x < map_width; x++){
      for (int y = 0; y < map_height; y++){
        if(map_cv1.at<unsigned char>(x, y) == CV_FREE){
          total_free_cells_on_maps[1]++;
        }
      }
    }
    for (int x = 0; x < map_width; x++){
      for (int y = 0; y < map_height; y++){
        if(map_cv2.at<unsigned char>(x, y) == CV_FREE){
          total_free_cells_on_maps[2]++;
        }
      }
    }

  //перебираем свободные клетки в списке
  for(int cell = 0; cell < free_cells.size(); cell++){
    bool this_robots_use_global_map[ROBOT_AMOUNT] = {false, false, false};
    Point start = free_cells[cell];
    vector<Point> borders;
    //создаем массив в который будем записывать числа (вектор на n строк, m элементов)
    int x_size = map_width;
    int y_size = map_height;
    vector < vector <int> > my_cost_map(x_size, vector <int> (y_size) );
    int total_free_cells_on_map = 0;
    int free_cells_amount = 0;

    
    //запишем в этот массив стены, неизвестные точки
    for (int x = 0; x < x_size; x++){
      for (int y = 0; y < y_size; y++){
        if (map_cv.at<unsigned char>(x, y) == CV_UNKNOWN){
          my_cost_map[x][y] = MYCOST_UNKNOWN; //неизвестная клетка
        } else if (map_cv.at<unsigned char>(x, y) == CV_OCCUPED){
          my_cost_map[x][y] = MYCOST_OCCUPED; //стена
        } else if (map_cv.at<unsigned char>(x, y) == CV_FREE){
          my_cost_map[x][y] = MYCOST_FREE; //свободная клетка
          total_free_cells_on_map++;
        }
      }
    }
    
    //будем считать число свободных клеток на других картах
    int free_cells_on_maps[3] = {0, 0, 0};

    //смешение относительно текущей точки
    const int dx[] = {0,1,0,-1};
    const int dy[] = {-1,0,1,0};
    int nstep = 0;

    //создаем волну 1 и волну 2. По очереди в них будем записывать координаты клеток наших волн
    vector <Point> wave_1;
    vector <Point> wave_2;
    vector < vector <Point> > waves = {wave_1,wave_2};
    int main_wave = 0;
    int second_wave = 1;
    
    my_cost_map[start.x][start.y] = 0;
    waves[main_wave].push_back(Point(start.x,start.y));



    //сюда будем записывать координаты, чтобы не обращаться к одним и тем же элементам векторов много раз
    int coord_x = 0;
    int coord_y = 0;

    

    while(1){
      
      waves[second_wave].clear();
      nstep++;
      //расширяем в 4 направления точки из первой волны. Новые записываем во вторую
      for (int i = 0; i < waves[main_wave].size(); i++){
        for (int side = 0; side < 4; side++){

          //записываем координаты
          coord_x = waves[main_wave][i].x + dx[side];
          coord_y = waves[main_wave][i].y + dy[side];

          //если это свободная клетка
          if(my_cost_map[coord_x][coord_y] == MYCOST_FREE){
            //сверяем с другими картами {
              if(map_cv0.at<unsigned char>(coord_x, coord_y) == CV_FREE){
                free_cells_on_maps[0]++;
              }
              if(map_cv1.at<unsigned char>(coord_x, coord_y) == CV_FREE){
                free_cells_on_maps[1]++;
              }
              if(map_cv2.at<unsigned char>(coord_x, coord_y) == CV_FREE){
                free_cells_on_maps[2]++;
              }
            // сверяем с другими картами}
            my_cost_map[coord_x][coord_y] = nstep; //присваиваем значение шага
            waves[second_wave].push_back(Point(coord_x, coord_y)); //заносим в волну координаты свободной точки
            free_cells_amount++;
            //если это неизвестная клетка
          } else if (my_cost_map[coord_x][coord_y] == MYCOST_UNKNOWN){
            //приверяем, что в массиве границы нет такой точки
            bool border_is_not_in_list = true;
            for (int k = 0; k < borders.size(); k++){
              if ((borders[k].x == coord_x) && (borders[k].y == coord_y)){
                //если граница оказывается в списке, то тогда мы ее не будем добавлять
                border_is_not_in_list = false;
              }
            }

            if (border_is_not_in_list){
              borders.push_back(Point(coord_x, coord_y));
            } 
          }
        }
      }

      swap(main_wave, second_wave);

      //если 2 волны пустые, то заканчиваем цикл
      if ((waves[main_wave].size() == 0) && (waves[second_wave].size() == 0)){
        int this_global_users_amount = 0;
        for (int r = 0; r < ROBOT_AMOUNT; r++){
          if ((float)free_cells_on_maps[r] > (0.6*(float)total_free_cells_on_maps[r])){
            this_robots_use_global_map[r] = true;
            this_global_users_amount++;
          }
        }
        if((this_global_users_amount >= 2) && (this_global_users_amount > best_global_users_amount)){
          //если это лучшая комбинация, то записываем ее
          for (int r = 0; r < ROBOT_AMOUNT; r++){
            best_robots_use_global_map[r] = this_robots_use_global_map[r];
          }
          best_global_users_amount = this_global_users_amount;
          best_borders = borders;
        }
        if (best_global_users_amount == 3){
          ROS_ERROR("all free robots unite maps");
          for (int r = 0; r < ROBOT_AMOUNT; r++){
            robot_use_local_map[r] = !best_robots_use_global_map[r];
          }
          return best_borders;
        }
        //переходим к след циклу
        goto Leave;
        
        /*
        if(free_cells_amount > (total_free_cells_on_map / 2)){
          //выходим из функции и возврашаем список неизвестных точек, которые граничат с известными
          ROS_INFO ("half of free area found");
          return borders;
        } else { //область очень мала.
          ROS_INFO ("area is too small");
          goto Leave;
        }
        */
      }
    }
    //выход из гоуту
    Leave:
    continue;
  }
  for (int r = 0; r < ROBOT_AMOUNT; r++){
    robot_use_local_map[r] = !best_robots_use_global_map[r];
  }
  return best_borders;
  //Вроде в этом месте возникает Варнинг, и компилятор достигает конца функции, но на деле проблем не было
}




 //от разных границ функция пускает волну и ставит точку на волне где-нибудь в неизвестности
 //эта волна может перекрыть соседние границы и от них тогда не будет пускаться волна
 //если волна достаточная большая не получилась, то мы ее игнорируем

void make_targets_from_borders(vector <Point> borders){ 

  int target_deep = global_target_deep;

  int x_size = map_width;
  int y_size = map_height;
  vector < vector <int> > my_cost_map(x_size, vector <int> (y_size) );


  //запишем в этот массив стены, неизвестные точки
  for (int x = 0; x < x_size; x++){
    for (int y = 0; y < y_size; y++){
      if (map_cv.at<unsigned char>(x, y) == CV_UNKNOWN){
        my_cost_map[x][y] = MYCOST_UNKNOWN; //неизвестная клетка
      } else if (map_cv.at<unsigned char>(x, y) == CV_OCCUPED){
        my_cost_map[x][y] = MYCOST_OCCUPED; //стена
      } else if (map_cv.at<unsigned char>(x, y) == CV_FREE){
        my_cost_map[x][y] = MYCOST_FREE; //свободная клетка
      }
    }
  }



  //смешение относительно текущей точки
    const int dx[] = {0,1,0,-1};
    const int dy[] = {-1,0,1,0};

  //для каждой точки на границе будем делать волну, волна будет превращать точки в занятые (OCCUPED)
  for (int bord = 0; bord < borders.size(); bord++){
    //если эта точка уже занята другой волной, то пропускаем цикл с этой точкой
    if (my_cost_map[borders[bord].x][borders[bord].y] == CV_OCCUPED){
      continue;
    }

    Point start = borders[bord];

    int nstep = 0;
    //создаем волну 1 и волну 2. По очереди в них будем записывать координаты клеток наших волн
    vector <Point> wave_1;
    vector <Point> wave_2;
    vector < vector <Point> > waves = {wave_1,wave_2};
    int main_wave = 0;
    int second_wave = 1;
    
    my_cost_map[start.x][start.y] = 0;
    waves[main_wave].push_back(Point(start.x,start.y));

    //сюда будем записывать координаты, чтобы не обращаться к одним и тем же элементам векторов много раз
    int coord_x = 0;
    int coord_y = 0;

    Point possible_target;
    while(1){
      
      
      waves[second_wave].clear();
      nstep++;
      //расширяем в 4 направления точки из первой волны. Новые записываем во вторую
      for (int i = 0; i < waves[main_wave].size(); i++){
        for (int side = 0; side < 4; side++){

          //записываем координаты
          coord_x = waves[main_wave][i].x + dx[side];
          coord_y = waves[main_wave][i].y + dy[side];

          if ((coord_x >= x_size) || (coord_x <= 0)){
            continue;
          }
          if ((coord_y >= y_size) || (coord_y <= 0)){
            continue;
          }

          //если это неизвестная
          if(my_cost_map[coord_x][coord_y] == MYCOST_UNKNOWN){
            my_cost_map[coord_x][coord_y] = nstep; //присваиваем значение шага
            waves[second_wave].push_back(Point(coord_x, coord_y)); //заносим в волну координаты свободной точки
          } 
        }
      }
      
      if (nstep == (target_deep/2)){
        possible_target = waves[second_wave][waves[second_wave].size()/2];
        //ROS_INFO("PUT TARGET POSSIBLE");
      }

      //если 2 волны пустые, то заканчиваем цикл
      if ((waves[main_wave].size() == 0) && (waves[second_wave].size() == 0)){
        //если набрали минимальное число шагов, то тогда вносим эту цель в список целей
        if(nstep > target_deep){
          //all_targets.push_back(Point(waves[second_wave][waves[second_wave].size()/2].x, waves[second_wave][waves[second_wave].size()/2].y));
          all_targets.push_back(possible_target);
          //ROS_INFO("PUT TARgET IN LIST");
        }
        goto end_cycle;
      }
      //если дошли до определенного числа шагов в волне
      if (nstep > target_deep){
        //вносимв список целей
        //all_targets.push_back(Point(waves[second_wave][waves[second_wave].size()/2].x, waves[second_wave][waves[second_wave].size()/2].y));
        all_targets.push_back(possible_target);
        //ROS_INFO("PUT TARgET IN LIST");
        //выходим из цикла и переходим к следюущей точке в границе
        goto end_cycle;
      }
      //менем волны местами
      swap(main_wave, second_wave);

    }
    end_cycle:
    continue;
    

  }

  //ROS_INFO("End of function make_targets_from_borders");
}



void find_way_to_all_targets (int robot_index){ //функция в объект каждого робота записывает расстояние до каждой цели
  //от этого робота пускаем волну
  int x_size = map_width;
  int y_size = map_height;
  vector < vector <int> > my_cost_map(x_size, vector <int> (y_size) );

  int targets_amount_overall = all_targets.size();
  int targets_found = 0;
  //запишем в этот массив стены, неизвестные точки
  for (int x = 0; x < x_size; x++){
    for (int y = 0; y < y_size; y++){
      if (map_cv.at<unsigned char>(x, y) == CV_UNKNOWN){
        my_cost_map[x][y] = MYCOST_FREE; //неизвестная клетку специально записываем как известную, чтобы считать путь
      } else if (map_cv.at<unsigned char>(x, y) == CV_OCCUPED){
        my_cost_map[x][y] = MYCOST_OCCUPED; //стена
      } else if (map_cv.at<unsigned char>(x, y) == CV_FREE){
        my_cost_map[x][y] = MYCOST_FREE; //свободная клетка
      }
    }
  }

  //вносим на карту все цели
  for(int i = 0; i < all_targets.size(); i++){
    my_cost_map[all_targets[i].x][all_targets[i].y] = MYCOST_TARGET;
  }

  const int dx[] = {0,1,0,-1};
  const int dy[] = {-1,0,1,0};

  Point start = myrobots[robot_index].pose_on_map_in_pix;

  int nstep = 0;
  //создаем волну 1 и волну 2. По очереди в них будем записывать координаты клеток наших волн
  vector <Point> wave_1;
  vector <Point> wave_2;
  vector < vector <Point> > waves = {wave_1,wave_2};
  int main_wave = 0;
  int second_wave = 1;
  
  my_cost_map[start.x][start.y] = 0;
  waves[main_wave].push_back(Point(start.x,start.y));

  //сюда будем записывать координаты, чтобы не обращаться к одним и тем же элементам векторов много раз
  int coord_x = 0;
  int coord_y = 0;

  //Point possible_target;  

   while(1){
      
      
    waves[second_wave].clear();
    nstep++;
    //расширяем в 4 направления точки из первой волны. Новые записываем во вторую
    for (int i = 0; i < waves[main_wave].size(); i++){
      for (int side = 0; side < 4; side++){

        //записываем координаты
        coord_x = waves[main_wave][i].x + dx[side];
        coord_y = waves[main_wave][i].y + dy[side];

        if ((coord_x >= x_size) || (coord_x <= 0)){
          continue;
        }
        if ((coord_y >= y_size) || (coord_y <= 0)){
          continue;
        }

      //если это свободная
        if(my_cost_map[coord_x][coord_y] == MYCOST_FREE){
          my_cost_map[coord_x][coord_y] = nstep; //присваиваем значение шага
          waves[second_wave].push_back(Point(coord_x, coord_y)); //заносим в волну координаты свободной точки
        } else if (my_cost_map[coord_x][coord_y] == MYCOST_TARGET){

          my_cost_map[coord_x][coord_y] = MYCOST_FREE; //обозначаем найденную точку свободной
          targets_found++;
          myrobots[robot_index].way_to_targets.push_back(nstep);
          myrobots[robot_index].targets_coord.push_back(Point(coord_x, coord_y));
          ROS_INFO("tb3_%d way will take %d to the x = %d, y = %d", robot_index, nstep, coord_x, coord_y);
        }
      }
    }
    if (targets_amount_overall == targets_found){
      ROS_INFO("ALL TARGETS FOUND BY ROBOT tb3_%d", robot_index);
      return;
    }
    
    if ((waves[main_wave].size() == 0) && (waves[second_wave].size() == 0)){
      ROS_ERROR("NOT ALL TARGETS WERE FOUND BY ROBOT tb3_%d", robot_index);
      return;
    }

    //менем волны местами
    swap(main_wave, second_wave);

  }


}


 
//если площадь окажется мала, мы ее запишем в этот массив и закрасим
vector<Point> temp_area;

//передав неизветную точку, мы выполним волну от этой точки с заданной глубиной, вернет середину этой волны
Point make_wave_in_unknown_area(Point start, int personal_target_deep){
  //cout << "making wave in unknown area" << endl;
  //int target_deep = global_target_deep;

  int x_size = map_width;
  int y_size = map_height;
  vector < vector <int> > my_cost_map(x_size, vector <int> (y_size));

  //запишем в этот массив стены, неизвестные точки
  for (int x = 0; x < x_size; x++){
    for (int y = 0; y < y_size; y++){
      if (map_cv.at<unsigned char>(x, y) == CV_UNKNOWN){
        my_cost_map[x][y] = MYCOST_UNKNOWN; //неизвестная клетка
      } else if (map_cv.at<unsigned char>(x, y) == CV_OCCUPED){
        my_cost_map[x][y] = MYCOST_OCCUPED; //стена
      } else if (map_cv.at<unsigned char>(x, y) == CV_FREE){
        my_cost_map[x][y] = MYCOST_FREE; //свободная клетка
      }
    }
  }

  //смешение относительно текущей точки
  const int dx[] = {0,1,0,-1};
  const int dy[] = {-1,0,1,0};

  int nstep = 0;
    //создаем волну 1 и волну 2. По очереди в них будем записывать координаты клеток наших волн
  vector <Point> wave_1;
  vector <Point> wave_2;
  vector < vector <Point> > waves = {wave_1,wave_2};
  int main_wave = 0;
  int second_wave = 1;
    
  my_cost_map[start.x][start.y] = 0;
  waves[main_wave].push_back(Point(start.x,start.y));
  temp_area.push_back(Point(start.x,start.y));

  //сюда будем записывать координаты, чтобы не обращаться к одним и тем же элементам векторов много раз
  int coord_x = 0;
  int coord_y = 0;

  Point possible_target;

  while(1){
    waves[second_wave].clear();
    nstep++;
    //расширяем в 4 направления точки из первой волны. Новые записываем во вторую
    for (int i = 0; i < waves[main_wave].size(); i++){
      for (int side = 0; side < 4; side++){

        //записываем координаты
        coord_x = waves[main_wave][i].x + dx[side];
        coord_y = waves[main_wave][i].y + dy[side];

        if ((coord_x >= x_size) || (coord_x <= 0)){
          continue;
        }
        if ((coord_y >= y_size) || (coord_y <= 0)){
          continue;
        }

        //если это неизвестная
        if(my_cost_map[coord_x][coord_y] == MYCOST_UNKNOWN){
         my_cost_map[coord_x][coord_y] = nstep;
         waves[second_wave].push_back(Point(coord_x, coord_y)); //заносим в волну координаты свободной точки
         temp_area.push_back(Point(coord_x, coord_y));
        } 
      }
    }
    
    
    
    if ((waves[main_wave].size() == 0) && (waves[second_wave].size() == 0)){
      //cout << "both waves = 0;" << endl; 
    //если обе волны равны нулю раньше времени, то возвращаем стартовую точку
    return start;
    }
    if (nstep > personal_target_deep){ //если мы дошли до 41 шага
    //cout << "got 40 steps sucsess" << endl;

      
      //ТАК БЫЛО ДО ВЫБОРА САМОЙ ДАЛЬНЕЙ НЕИЗВЕСТНОЙ ТОЧКИ В ВОЛНЕ
      //делим волну пополам и возвращаем середину волны
      temp_area.clear();
      return waves[second_wave][waves[second_wave].size()/2];
      
    }

    swap(main_wave, second_wave);
  }

}




//Функция находит ближайшую неизвестную область от конкретного робота
Point find_closest_big_unknown_area(Point start){ 
  //cout << "find_closest_big_unknown_area" << endl;
  //int target_deep = global_target_deep;
  int personal_target_deep = 40;
  int try_amount = 20; //если в течении 100 попыток не нашли нужную площадь
  int x_size = map_width;
  int y_size = map_height;
  vector < vector <int> > my_cost_map(x_size, vector <int> (y_size));

  //запишем в этот массив стены, неизвестные точки
  for (int x = 0; x < x_size; x++){
    for (int y = 0; y < y_size; y++){
      if (map_cv.at<unsigned char>(x, y) == CV_UNKNOWN){
        my_cost_map[x][y] = MYCOST_UNKNOWN; //неизвестная клетка
      } else if (map_cv.at<unsigned char>(x, y) == CV_OCCUPED){
        my_cost_map[x][y] = MYCOST_OCCUPED; //стена
      } else if (map_cv.at<unsigned char>(x, y) == CV_FREE){
        my_cost_map[x][y] = MYCOST_FREE; //свободная клетка
      }
    }
  }

  //смешение относительно текущей точки
  const int dx[] = {0,1,0,-1};
  const int dy[] = {-1,0,1,0};

  int nstep = 0;
    //создаем волну 1 и волну 2. По очереди в них будем записывать координаты клеток наших волн
  vector <Point> wave_1;
  vector <Point> wave_2;
  vector < vector <Point> > waves = {wave_1,wave_2};
  int main_wave = 0;
  int second_wave = 1;
    
  my_cost_map[start.x][start.y] = 0;
  waves[main_wave].push_back(Point(start.x,start.y));

  //сюда будем записывать координаты, чтобы не обращаться к одним и тем же элементам векторов много раз
  int coord_x = 0;
  int coord_y = 0;

  Point possible_target;

  while(1){
    waves[second_wave].clear();
    nstep++;
    //расширяем в 4 направления точки из первой волны. Новые записываем во вторую
    for (int i = 0; i < waves[main_wave].size(); i++){
      for (int side = 0; side < 4; side++){

        //записываем координаты
        coord_x = waves[main_wave][i].x + dx[side];
        coord_y = waves[main_wave][i].y + dy[side];

        if ((coord_x >= x_size) || (coord_x <= 0)){
          continue;
        }
        if ((coord_y >= y_size) || (coord_y <= 0)){
          continue;
        }

        //если это известная
        if(my_cost_map[coord_x][coord_y] == MYCOST_FREE){
          my_cost_map[coord_x][coord_y] = nstep;
          waves[second_wave].push_back(Point(coord_x, coord_y)); //заносим в волну координаты свободной точки
        } else if (my_cost_map[coord_x][coord_y] == MYCOST_UNKNOWN){
          Point possible_target(coord_x,coord_y);
          //проверяем площадь
          Point unknown_area_target = make_wave_in_unknown_area(possible_target, personal_target_deep);
          if ((unknown_area_target.x != possible_target.x)&& (unknown_area_target.y != possible_target.y)){
            cout << "return turget" << endl;
            return unknown_area_target;
          } else {
            //если площадь мала, то заполняем ее пустыми ячейками
            for (int c = 0; c < temp_area.size(); c++){
              my_cost_map[temp_area[c].x][temp_area[c].y] = nstep;
              waves[second_wave].push_back(Point(temp_area[c].x, temp_area[c].y));
            }
            temp_area.clear();
            /*
            //площадь мала, выставляем значение точки как nstep
            my_cost_map[coord_x][coord_y] = nstep;
            waves[second_wave].push_back(Point(coord_x, coord_y));
            */
            try_amount--;
            if (try_amount < 0){ //если в течении 20 попыток не нашли нужную по размеру область, то едем к неизв точке.
              return possible_target;
            }
            cout << "CELL [" << coord_x << "][" << coord_y << "] set as free"<< endl;
          }
        }
      }
    }
    //менем волны местами
    swap(main_wave, second_wave);
    if ((waves[main_wave].size() == 0) && (waves[second_wave].size() == 0)){ 
    //если обе волны равны нулю раньше времени
    //тогда неизвестных точек нет
    ROS_ERROR("PERSONAL TARGET NOT FOUND");
    return start;
    }
  }
}




//Выбирает персональную цель для робота
Point find_new_personal_target_for_robot(int robot_index){
  Point robot_target = find_closest_big_unknown_area(myrobots[robot_index].pose_on_map_in_pix);
  return robot_target;
}

//Функция ищет роботов которые готовы к выполнению задач.
vector<int> is_robots_in_free_area(){ //функция возвращает массив индеков роботов, которые находятся в известном свободном пространстве

  //робот считается в неизвестном пространстве:
  //  -  если 8 клеток вокруг его сейф зоны - неизвестное или стена то он в неизвестном (можно расширить до 16)
  //  -  если он находится к какой-либо цели ближе чем на 10 шагов
  vector<int> ready_for_tasks_robots;
  
  //смешение относительно текущей точки
  const int dx[] = {2, -2, 2, -2, 2, -2, 0, 0};
  const int dy[] = {2, -2, -2, 2, 0, 0, 2, -2};

  for (int r = 0; r < ROBOT_AMOUNT; r++){
    //по умолчанию выставляем что персональная цель не нужна
    myrobots[r].need_personal_target = false;
    bool need_pers_target = true; //ставим мол цель нужна, если нашли известную клетку вокруг нашли, то не нужна 

    //если робот испольует свою карту то ему нужна персональная цель
    if(robot_use_local_map[r]){
      myrobots[r].need_personal_target = true;
      continue; //пропускаем этого робота
    }

    //если робот застрял, то ему нужна собственная цель
    if (myrobots[r].stuck){
      myrobots[r].need_personal_target = true;
      continue; //пропускаем этого робота
    }
  
    //проверяем 8 клеток
    for(int side = 0; side < 8; side++){
      if (map_cv.at<unsigned char>(myrobots[r].pose_on_map_in_pix.x + dx[side], myrobots[r].pose_on_map_in_pix.y + dy[side]) == CV_FREE){
        need_pers_target = false;
      }
    }
    if (need_pers_target == true){
      myrobots[r].need_personal_target = true;
      myrobots[r].clear_ways_and_targets_coord();
      ROS_WARN("tb3_%d is in unknown area (got personal target)", r);
      continue; // обработка для этого робота завершена, переходим к следующему
      
    }
    

    //проверяем близость к одной из целей
    for(int i = 0; i < myrobots[r].way_to_targets.size(); i++){
      if (myrobots[r].way_to_targets[i] < 10){
        need_pers_target = true;
      }
    }
    if (need_pers_target == true){
      myrobots[r].need_personal_target == true;
      myrobots[r].clear_ways_and_targets_coord();
      ROS_WARN("tb3_%d is too close to any target (got personal target)", r);
      continue; // обработка для этого робота завершена, переходим к следующему
      
    } else {
      ready_for_tasks_robots.push_back(r);
    }
  }

  return ready_for_tasks_robots;
}

//Функция возвращает таблицу робот-путь до определенной цели
vector<vector<int>> create_way_table(vector<int> ready_for_tasks_robots){
  //создаем таблицу путей
  //роботы это первая координата, цели это вторая координата. Пересечение координат дает расстояние до цели
  vector<vector<int>> way_table(ready_for_tasks_robots.size(), vector <int> (all_targets.size()));
  
  //заполняем таблицу
  for (int r = 0; r < ready_for_tasks_robots.size(); r++){
    //ИМЕНА РОБОТОВ НАХОДЯТСЯ В СПИСКЕ ГОТОВЫХ К ЗАДАЧАМ
    for (int i = 0; i < all_targets.size(); i++){
      for (int k = 0; k < myrobots[ready_for_tasks_robots[r]].targets_coord.size(); k++){
        if ( (all_targets[i].x == myrobots[ready_for_tasks_robots[r]].targets_coord[k].x)  && (all_targets[i].y == myrobots[ready_for_tasks_robots[r]].targets_coord[k].y)){
          //цели совпали, записываем в таблицу
          way_table[r][i] = myrobots[ready_for_tasks_robots[r]].way_to_targets[k];
        }
      }
    }
    //очищаем список координат целей и список путей целей
    myrobots[ready_for_tasks_robots[r]].clear_ways_and_targets_coord();
  }
  return way_table;
}


class Combination {
  public:
    //комбинация, в которой содержится номер цели для каждого робота. То есть этот вектор всегда по длине это число роботов.
    vector<int> combination_robot_target;
    //вектор суммы путей этой же комбинации
    int way_sum;
};


//рекурсивная функция для выбора лучшей комбинации
void target_combinations_recursion(vector<vector<int>> way_table, vector<int> this_combination, int targets_minus_robots, int target_amount,int ready_robots_amount, vector<Combination> &combinations_and_ways){
  bool this_is_last_cycle = false;
  int empty_targets = 0;
  if (this_combination.size() == (ready_robots_amount-1)){
    //значит это последний цикл
    this_is_last_cycle = true;
  }
  if (targets_minus_robots < 0){
    empty_targets = targets_minus_robots;
  }
  
  for (int t = empty_targets; t <  target_amount; t++){

    //проверяем, что эту цель еще не взяли
    for (int k = 0; k < this_combination.size(); k++){
      //если эту цель взяли, то пропускаем цикл
      if (this_combination[k] == t){
        goto skip_cycle;
      }
    }


    this_combination.push_back(t);
    if (this_is_last_cycle){
      //внести комбинацию в список
      Combination comb_and_way;
      comb_and_way.combination_robot_target = this_combination;
      //combinations.push_back(this_combination);
      //внести суммарный путь при этой комбинации
      int way_sum_for_this_combination = 0;
      for (int i = 0; i < ready_robots_amount; i++){

        if (this_combination[i] < 0){ //если это пустая цель, то пропускаем
          continue;
        }
        way_sum_for_this_combination = way_sum_for_this_combination + way_table[i][this_combination[i]];
      }
      comb_and_way.way_sum = way_sum_for_this_combination;
      combinations_and_ways.push_back(comb_and_way);
      //way_sums.push_back(way_sum_for_this_combination);
    } else {
      //идем ниже по рекурсии
      target_combinations_recursion(way_table, this_combination, targets_minus_robots, target_amount, ready_robots_amount, combinations_and_ways);
    }
    //в любом случае в конце надо подтереть последний элемент в комбинации
    this_combination.pop_back();

    skip_cycle:
    continue;
  }
}

void set_personal_targets(){
   //выдаем персональные цели робота
  for (int r = 0; r < ROBOT_AMOUNT; r++){
    if (myrobots[r].need_personal_target){
      //myrobots[r].goal_on_map = find_closest_big_unknown_area(myrobots[r].pose_on_map_in_pix); удалить
      myrobots[r].goal_on_map = find_new_personal_target_for_robot(r);
      myrobots[r].need_personal_target = false;
      ROS_WARN("PERSONAL TARGET tb3_%d is x = %d, y = %d", r, myrobots[r].goal_on_map.x, myrobots[r].goal_on_map.y);
    }
  }
}




//сюда будет записываться последняя комбинация
vector <Point> last_robot_target_comb(ROBOT_AMOUNT, Point(999999, 999999));
//сюда будет записываться текущая комбинация
vector <Point> current_robot_target_comb(ROBOT_AMOUNT, Point(999999, 999999)); 


Combination keep_last_targets(vector<vector<int>> way_table, int target_amount, int ready_robots_amount, vector<int> ready_for_tasks_robots, vector<Combination> &combinations_and_ways){
  
  Combination common_to_last_combination;
  
  //если один робот
  if (ready_robots_amount == 1){
    return combinations_and_ways[0];
  }

  int deviation = (int)(combinations_and_ways[0].way_sum * DEVIATION_SIZE);
  
  //находим число лучших комбинаций, которые возможно использовать
  int best_combination_amount = 0;
  for (int i = 0; i < combinations_and_ways.size(); i++){
    if(combinations_and_ways[0].way_sum + deviation <= combinations_and_ways[i].way_sum){
      best_combination_amount = i;
      break;
    }

    if(i == (combinations_and_ways.size() - 1)){
      best_combination_amount = combinations_and_ways.size();
    }
  }

  //если всего одна комбинация, то используем ее
  if(best_combination_amount == 1){
    //cout << "only one good combnation"<< endl;
    return combinations_and_ways[0];
  }

  #ifdef MYDEBUG
  cout << "best combintaions" << endl;
  for (int i = 0; i < best_combination_amount; i++){
    cout << combinations_and_ways[i].way_sum << endl;
  }

   cout << "     " << endl;
  #endif //MYDEBUG

  
  int max_same_targets = 0;
  int best_comb_index = 0;
  

  for (int i = best_combination_amount-1; i >= 0 ; i--){
    int number_of_same_targets = 0;
    for (int r = 0; r < ready_robots_amount; r++){
      current_robot_target_comb[ready_for_tasks_robots[r]] = all_targets[combinations_and_ways[i].combination_robot_target[r]];
    }
    for (int r = 0; r < ROBOT_AMOUNT; r++){
      if ((current_robot_target_comb[r].x > (last_robot_target_comb[r].x - 10)) && (current_robot_target_comb[r].x < (last_robot_target_comb[r].x + 10))){
        if((current_robot_target_comb[r].y > (last_robot_target_comb[r].y - 10)) && (current_robot_target_comb[r].y < (last_robot_target_comb[r].y + 10))){
          number_of_same_targets++;
        }
      }
    }
    if (number_of_same_targets >= max_same_targets){
      max_same_targets = number_of_same_targets;
      best_comb_index = i;
    }

  }

  //записываем наулучшую комбинацию
  for (int r = 0; r < ready_robots_amount; r++){
    last_robot_target_comb[ready_for_tasks_robots[r]] = all_targets[combinations_and_ways[best_comb_index].combination_robot_target[r]];
  }

  #ifdef MYDEBUG
  ROS_INFO("%d best comb index",best_comb_index);
  #endif //MYDEBUG

  return combinations_and_ways[best_comb_index];
}




//функция ставит на карте точки на определенном расстояниии от робота
//если не получилось поставить точки, вернет пустой вектор
//если получилось, то вернет норм вектор с этими точками
vector <Point> make_aura_point_around_robot(Point start, int dist){
  int x_size = map_width;
  int y_size = map_height;
  vector < vector <int> > my_cost_map(x_size, vector <int> (y_size) );

  //запишем в этот массив стены, неизвестные точки
  for (int x = 0; x < x_size; x++){
    for (int y = 0; y < y_size; y++){
      if (map_cv.at<unsigned char>(x, y) == CV_UNKNOWN){
        my_cost_map[x][y] = MYCOST_UNKNOWN; //неизвестная клетку специально записываем как известную, чтобы считать путь
      } else if (map_cv.at<unsigned char>(x, y) == CV_OCCUPED){
        my_cost_map[x][y] = MYCOST_OCCUPED; //стена
      } else if (map_cv.at<unsigned char>(x, y) == CV_FREE){
        my_cost_map[x][y] = MYCOST_FREE; //свободная клетка
      } 
    }
  }

  //смешение относительно текущей точки
  const int dx[] = {0,1,0,-1};
  const int dy[] = {-1,0,1,0};

  int nstep = 0;
    //создаем волну 1 и волну 2. По очереди в них будем записывать координаты клеток наших волн
  vector <Point> wave_1;
  vector <Point> wave_2;
  vector < vector <Point> > waves = {wave_1,wave_2};
  int main_wave = 0;
  int second_wave = 1;
    
  my_cost_map[start.x][start.y] = 0;
  waves[main_wave].push_back(Point(start.x,start.y));

  //сюда будем записывать координаты, чтобы не обращаться к одним и тем же элементам векторов много раз
  int coord_x = 0;
  int coord_y = 0;



  while(1){
    waves[second_wave].clear();
    nstep++;
    //расширяем в 4 направления точки из первой волны. Новые записываем во вторую
    for (int i = 0; i < waves[main_wave].size(); i++){
      for (int side = 0; side < 4; side++){

        //записываем координаты
        coord_x = waves[main_wave][i].x + dx[side];
        coord_y = waves[main_wave][i].y + dy[side];

        if ((coord_x >= x_size) || (coord_x <= 0)){
          continue;
        }
        if ((coord_y >= y_size) || (coord_y <= 0)){
          continue;
        }

        //если это известная
        if(my_cost_map[coord_x][coord_y] == MYCOST_FREE){
          my_cost_map[coord_x][coord_y] = nstep;
          waves[second_wave].push_back(Point(coord_x, coord_y)); //заносим в волну координаты свободной точки
        } 
      }
    }
    //если есть 70 шагов, возвращаем последнюю волну
    if (nstep == 70){
      return waves[second_wave];
    }


    //менем волны местами
    swap(main_wave, second_wave);
    
    //если волны закончились, то возвращаем пустую волну
    if ((waves[main_wave].size() == 0) && (waves[second_wave].size() == 0)){ 
      return waves[second_wave];
    }
  }


}


//фунция выберет точку цель тому роботу, которому необходимо отдалиться от своего товарища
Point get_away_from_robot(Point start, Point nearest_robot){
  int x_size = map_width;
  int y_size = map_height;
  vector < vector <int> > my_cost_map(x_size, vector <int> (y_size) );

  //запишем в этот массив стены, неизвестные точки
  for (int x = 0; x < x_size; x++){
    for (int y = 0; y < y_size; y++){
      if (map_cv.at<unsigned char>(x, y) == CV_UNKNOWN){
        my_cost_map[x][y] = MYCOST_UNKNOWN; //неизвестная клетку специально записываем как известную, чтобы считать путь
      } else if (map_cv.at<unsigned char>(x, y) == CV_OCCUPED){
        my_cost_map[x][y] = MYCOST_OCCUPED; //стена
      } else if (map_cv.at<unsigned char>(x, y) == CV_FREE){
        my_cost_map[x][y] = MYCOST_FREE; //свободная клетка
      } 
    }
  }

  vector<Point> aura = make_aura_point_around_robot(nearest_robot, 50);
  if(aura.size() == 0){
    ROS_ERROR("NO SPACE TO GET AWAY");
    return start; //тогда вернем просто позицию нашего робота
  } else {
    ROS_WARN("MOVE AWAY FROM ROBOT");
    for (int i = 0; i < aura.size(); i++){
      my_cost_map[aura[i].x][aura[i].y] = MYCOST_TARGET;
    }
  }



  const int dx[] = {0,1,0,-1};
  const int dy[] = {-1,0,1,0};

  int nstep = 0;
    //создаем волну 1 и волну 2. По очереди в них будем записывать координаты клеток наших волн
  vector <Point> wave_1;
  vector <Point> wave_2;
  vector < vector <Point> > waves = {wave_1,wave_2};
  int main_wave = 0;
  int second_wave = 1;
    
  my_cost_map[start.x][start.y] = 0;
  waves[main_wave].push_back(Point(start.x,start.y));

  //сюда будем записывать координаты, чтобы не обращаться к одним и тем же элементам векторов много раз
  int coord_x = 0;
  int coord_y = 0;

  while(1){
    waves[second_wave].clear();
    nstep++;
    //расширяем в 4 направления точки из первой волны. Новые записываем во вторую
    for (int i = 0; i < waves[main_wave].size(); i++){
      for (int side = 0; side < 4; side++){

        //записываем координаты
        coord_x = waves[main_wave][i].x + dx[side];
        coord_y = waves[main_wave][i].y + dy[side];

        if ((coord_x >= x_size) || (coord_x <= 0)){
          continue;
        }
        if ((coord_y >= y_size) || (coord_y <= 0)){
          continue;
        }

        //если это известная
        if(my_cost_map[coord_x][coord_y] == MYCOST_FREE){
          my_cost_map[coord_x][coord_y] = nstep;
          waves[second_wave].push_back(Point(coord_x, coord_y)); //заносим в волну координаты свободной точки
        } else if (my_cost_map[coord_x][coord_y] == MYCOST_TARGET){
          return Point(coord_x, coord_y);
        }
      }
    }
    //менем волны местами
    swap(main_wave, second_wave);

    if ((waves[main_wave].size() == 0) && (waves[second_wave].size() == 0)){ 
    //если обе волны равны нулю раньше времени, то возвращаем стартовую точку
    //то есть мы не нашли ни одного робота
    ROS_ERROR("NO ROBOT TO FOLLOW FOUND");
    return start;
    }
  }



}



//функция находит ближайшего робота с целью, за которым можно следовать
//функция возвращает точку на каком-то расстояниии до робота за которым будем следовать
Point find_robot_to_follow(int r_in_ready, vector<int> ready_for_tasks_robots ){
  int x_size = map_width;
  int y_size = map_height;
  vector < vector <int> > my_cost_map(x_size, vector <int> (y_size) );

  //запишем в этот массив стены, неизвестные точки
  for (int x = 0; x < x_size; x++){
    for (int y = 0; y < y_size; y++){
      if (map_cv.at<unsigned char>(x, y) == CV_UNKNOWN){
        my_cost_map[x][y] = MYCOST_UNKNOWN; //неизвестная клетку специально записываем как известную, чтобы считать путь
      } else if (map_cv.at<unsigned char>(x, y) == CV_OCCUPED){
        my_cost_map[x][y] = MYCOST_OCCUPED; //стена
      } else if (map_cv.at<unsigned char>(x, y) == CV_FREE){
        my_cost_map[x][y] = MYCOST_FREE; //свободная клетка
      } 
    }
  }

  
  
  //выставляем роботов на карте
  for(int r = 0; r < ready_for_tasks_robots.size(); r++){
    if (r == r_in_ready){ // если это этот робот, пропускаем
      continue;
    } else { //если робот может иметь последователей, то вносим его на карту
      if (myrobots[ready_for_tasks_robots[r]].robot_can_have_folowers){
        my_cost_map[myrobots[ready_for_tasks_robots[r]].pose_on_map_in_pix.x][myrobots[ready_for_tasks_robots[r]].pose_on_map_in_pix.y] = MYCOST_ROBOT - ready_for_tasks_robots[r];
      }
    }
  }
  

  //пускаем волну по известной области от нашего робота
  Point start(myrobots[ready_for_tasks_robots[r_in_ready]].pose_on_map_in_pix.x, myrobots[ready_for_tasks_robots[r_in_ready]].pose_on_map_in_pix.y);
  //смешение относительно текущей точки
  const int dx[] = {0,1,0,-1};
  const int dy[] = {-1,0,1,0};

  int nstep = 0;
    //создаем волну 1 и волну 2. По очереди в них будем записывать координаты клеток наших волн
  vector <Point> wave_1;
  vector <Point> wave_2;
  vector < vector <Point> > waves = {wave_1,wave_2};
  int main_wave = 0;
  int second_wave = 1;
    
  my_cost_map[start.x][start.y] = 0;
  waves[main_wave].push_back(Point(start.x,start.y));

  //сюда будем записывать координаты, чтобы не обращаться к одним и тем же элементам векторов много раз
  int coord_x = 0;
  int coord_y = 0;

  Point possible_target;
  Point nearest_robot;

  while(1){
    waves[second_wave].clear();
    nstep++;
    //расширяем в 4 направления точки из первой волны. Новые записываем во вторую
    for (int i = 0; i < waves[main_wave].size(); i++){
      for (int side = 0; side < 4; side++){

        //записываем координаты
        coord_x = waves[main_wave][i].x + dx[side];
        coord_y = waves[main_wave][i].y + dy[side];

        if ((coord_x >= x_size) || (coord_x <= 0)){
          continue;
        }
        if ((coord_y >= y_size) || (coord_y <= 0)){
          continue;
        }

        //если это известная
        if(my_cost_map[coord_x][coord_y] == MYCOST_FREE){
          my_cost_map[coord_x][coord_y] = nstep;
          waves[second_wave].push_back(Point(coord_x, coord_y)); //заносим в волну координаты свободной точки
        } else {
          for (int rb = 0; rb < ready_for_tasks_robots.size(); rb++) {
            if (my_cost_map[coord_x][coord_y] == MYCOST_ROBOT - ready_for_tasks_robots[rb]){
              //значит это цель
              nearest_robot = Point(coord_x,coord_y);
              myrobots[ready_for_tasks_robots[rb]].followers_amount++; //обозначаем что мы подписались на робота
              myrobots[ready_for_tasks_robots[r_in_ready]].pose_in_follow_queue = myrobots[ready_for_tasks_robots[rb]].followers_amount; //обозначаем наш номер в очереди при следовании за роботом
              //если мы будем первыми в очереди, то наш номер будет 1.

              goto find_way_to_robot;
            }
          }
        }
      }
    }
    //менем волны местами
    swap(main_wave, second_wave);

    if ((waves[main_wave].size() == 0) && (waves[second_wave].size() == 0)){ 
    //если обе волны равны нулю раньше времени, то возвращаем стартовую точку
    //то есть мы не нашли ни одного робота
    ROS_ERROR("NO ROBOT TO FOLLOW FOUND");
    return start;
    }
  }


  //ищем путь от этого робота до нашего
  find_way_to_robot:
  Point next_step_point = nearest_robot;
  int negativ_step = nstep;
  while(1){
    for (int side = 0; side < 4; side++){
      //записываем координаты
      coord_x = next_step_point.x + dx[side];
      coord_y = next_step_point.y + dy[side];
      if ((coord_x >= x_size) || (coord_x <= 0)){
        continue;
      }
      if ((coord_y >= y_size) || (coord_y <= 0)){
        continue;
      }

      if (my_cost_map[coord_x][coord_y] == (negativ_step - 1)){
        negativ_step--;
        next_step_point = Point(coord_x, coord_y);
      }
      if (my_cost_map[coord_x][coord_y] == 0){
        //нашли роботам самого же, возвращаем позицию робота просто
        ROS_WARN("TOO CLOSE TO ROBOT THAT FOLOW");
        if (GET_AWAY_FROM_FOLLOW_ROBOT){
          return get_away_from_robot(start, nearest_robot); //мы хотим отодвинуться от робота за которым следуем.
        } else {
          return start;
        }
      }
      if (negativ_step == (nstep - 70*myrobots[ready_for_tasks_robots[r_in_ready]].pose_in_follow_queue)){
        return next_step_point;
      }
    }
  }
}


//удаляет цель из вектора целей под определенным индеком
void delete_target(int target_index){
    vector<Point> temp_targets;
    for (int i = 0; i < all_targets.size(); i++){
        if (i == target_index){
            continue;
        }
        temp_targets.push_back(all_targets[i]);
    }
    all_targets = temp_targets;
}



//делает волну по известной и неизвестной области, доходя до цели удаляет ее из общего списка
void delete_closest_target_for_robot_in_unknown_space(int r){
  Point start = myrobots[r].pose_on_map_in_pix;
  int x_size = map_width;
  int y_size = map_height;
  vector < vector <int> > my_cost_map(x_size, vector <int> (y_size) );

  //запишем в этот массив стены, неизвестные точки
  for (int x = 0; x < x_size; x++){
    for (int y = 0; y < y_size; y++){
      if (map_cv.at<unsigned char>(x, y) == CV_UNKNOWN){
        my_cost_map[x][y] = MYCOST_FREE; //запишем как известную
      } else if (map_cv.at<unsigned char>(x, y) == CV_OCCUPED){
        my_cost_map[x][y] = MYCOST_OCCUPED; //стена
      } else if (map_cv.at<unsigned char>(x, y) == CV_FREE){
        my_cost_map[x][y] = MYCOST_FREE; //свободная клетка
      }
    }
  }

  //выставляем цели на карте
  for (int i = 0; i < all_targets.size(); i++){
    my_cost_map[all_targets[i].x][all_targets[i].y] = MYCOST_TARGET;
  }

  const int dx[] = {0,1,0,-1};
  const int dy[] = {-1,0,1,0};
  int nstep = 0;
    //создаем волну 1 и волну 2. По очереди в них будем записывать координаты клеток наших волн
  vector <Point> wave_1;
  vector <Point> wave_2;
  vector < vector <Point> > waves = {wave_1,wave_2};
  int main_wave = 0;
  int second_wave = 1;
    
  my_cost_map[start.x][start.y] = 0;
  waves[main_wave].push_back(Point(start.x,start.y));

  int coord_x = 0;
  int coord_y = 0;

  while(1){
    waves[second_wave].clear();
    nstep++;
    //расширяем в 4 направления точки из первой волны. Новые записываем во вторую
    for (int i = 0; i < waves[main_wave].size(); i++){
      for (int side = 0; side < 4; side++){

        //записываем координаты
        coord_x = waves[main_wave][i].x + dx[side];
        coord_y = waves[main_wave][i].y + dy[side];

        if ((coord_x >= x_size) || (coord_x <= 0)){
          continue;
        }
        if ((coord_y >= y_size) || (coord_y <= 0)){
          continue;
        }

        //если это известная
        if(my_cost_map[coord_x][coord_y] == MYCOST_FREE){
          my_cost_map[coord_x][coord_y] = nstep;
          waves[second_wave].push_back(Point(coord_x, coord_y)); //заносим в волну координаты свободной точки
        } else if (my_cost_map[coord_x][coord_y] == MYCOST_TARGET){
          //удяляем цель из списка
          for (int t = 0; t < all_targets.size(); t++){
            if ((coord_x == all_targets[t].x) && (coord_y == all_targets[t].y)){
              //если цель совпала, то нужно ее удалить
              delete_target(t);
              return;
            }
          }
          
        }
      }
    }
    //менем волны местами
    swap(main_wave, second_wave);

    if ((waves[main_wave].size() == 0) && (waves[second_wave].size() == 0)){ 
    //если обе волны равны нулю раньше времени, то возвращаем стартовую точку
    //то есть мы не нашли ни одного робота
    ROS_ERROR("NO TARGET TO DELETE FOUND");
    return;
    }
  }
}


















//ГЛАВНАЯ ФУНКЦИЯ ВЫДАЧИ ЦЕЛЕЙ РОБОТАМ
void give_targets_to_robots(){
//ROS_WARN("GLOBAL TARGET DEEP = %d", global_target_deep);
  if(all_targets.size() == 0){
    ROS_ERROR("TARGETS NOT FOUND (function give_targets_to_robots)");
    
    //Уменьшаем минимальное требование к цели
    if(global_target_deep > 30){
      global_target_deep = global_target_deep - 10;
    }
    return;
  }

  //ROS_INFO("will find how many robots are in unknown space");
  //проверяем сколько у нас роботов в известном пространстве. Если в неизвестном, то в распределении целей они не участвуют
  vector<int> ready_for_tasks_robots = is_robots_in_free_area();
  int ready_robots_amount = ready_for_tasks_robots.size();

  if (ready_robots_amount == 0){
    ROS_WARN("ALL ROBOTS ARE BUSY (in unknown area)");
    set_personal_targets();
    return;
  }
  ROS_INFO("%d robots are ready for targets", ready_robots_amount);


  //находим для каждого робота в неизвестной области ближайшую общую цель и удаляем ее из списка целей.
  for (int r = 0; r < ROBOT_AMOUNT; r++){
    if(myrobots[r].need_personal_target){
      if (myrobots[r].stuck == false){ //если робот застрял, то удалять ближайшую цель нельзя
        if (robot_use_local_map[r] == false){
        delete_closest_target_for_robot_in_unknown_space(r);
        }
      }
    }
  }


  //выдаем персональные цели
  set_personal_targets();


  

  //ROS_INFO("will create way table");
  //создаем таблицу путей
  //роботы это первая координата, цели это вторая координата. Пересечение координат дает расстояние до цели
  vector<vector<int>> way_table = create_way_table(ready_for_tasks_robots);

  //ROS_INFO("way table is created");

  

  //создаем вектор комбинации
  vector<Combination> combinations_and_ways;

  

  //заносим в этот вектор варианты
  int target_amount = all_targets.size();
  int targets_minus_robots = target_amount - ready_robots_amount;


  //ROS_INFO("will find all combinations");
  //находим все комбинации и их суммы, записываем в combinations и way_sums
  vector<int> this_combination;
  target_combinations_recursion(way_table, this_combination, targets_minus_robots, target_amount, ready_robots_amount, combinations_and_ways);

  //ROS_INFO("number of combination found %ld", combinations_and_ways.size());

  //ROS_INFO("will sort results");
  //сортируем результаты
  sort(combinations_and_ways.begin(), combinations_and_ways.end(), [](const Combination &p1, const Combination &p2){
    return p1.way_sum < p2.way_sum;
  });

  //DEBUG{
  //cout << " SUMS OF WAYS " << endl;
  //for (int i = 0; i < combinations_and_ways.size(); i++){
    //cout << combinations_and_ways[i].way_sum << endl;
  //}
  //DEBUG}

  //ROS_INFO("results are sorted");



//находим комбинации, чтобы прежние цели совпадали
  Combination best_targets = keep_last_targets(way_table, target_amount, ready_robots_amount, ready_for_tasks_robots, combinations_and_ways);

    //извлекаем лучшие комбинации и выдаем роботам
  for (int r = 0; r < ready_for_tasks_robots.size(); r++){
    if(best_targets.combination_robot_target[r] < 0){
      myrobots[ready_for_tasks_robots[r]].goal_on_map = myrobots[ready_for_tasks_robots[r]].pose_on_map_in_pix;
    } else {
      myrobots[ready_for_tasks_robots[r]].goal_on_map = all_targets[best_targets.combination_robot_target[r]];
      //выставляем переменные роботу
      myrobots[ready_for_tasks_robots[r]].robot_can_have_folowers = true;
      myrobots[ready_for_tasks_robots[r]].followers_amount = 0;
      myrobots[ready_for_tasks_robots[r]].pose_in_follow_queue = 0;
    }
  }

   //Присваиваем цели роботом, у которых нет цели и они следуют за товарищами
  for (int r = 0; r < ready_for_tasks_robots.size(); r++){
    if(best_targets.combination_robot_target[r] < 0){ 
      myrobots[ready_for_tasks_robots[r]].goal_on_map = find_robot_to_follow(r, ready_for_tasks_robots);
    }
  }

  //сбрасываем переменные 
  for (int r = 0; r < ROBOT_AMOUNT; r++){
    myrobots[r].robot_can_have_folowers = false;
    myrobots[r].followers_amount = 0;
    myrobots[r].pose_in_follow_queue = 0;
  }


}


//калибровка, первый робот объезжает нулевого
bool calibration_in_progress = CALIBRATE_ROBOTS;

void merge_calibration(){ //робот 1 должет объехать свою группу слева, это повысит общую зону перекрытия
  //калибровка объединения карты
  if (calibration_in_progress){
    for (int r = 0; r < ROBOT_AMOUNT; r++){
      myrobots[r].goal_on_map = myrobots[r].pose_on_map_in_pix;
    }
    //выставляем цель слева от нулевого робота
    
    myrobots[1].goal_on_map.x = myrobots[0].pose_on_map_in_pix.x;
    myrobots[1].goal_on_map.y = myrobots[0].pose_on_map_in_pix.y + 20; //+30
    
    //проверка что цель достигнута
    if ((myrobots[1].pose_on_map_in_pix.x > (myrobots[1].goal_on_map.x - 10)) && (myrobots[1].pose_on_map_in_pix.x < (myrobots[1].goal_on_map.x + 10))){
      if ((myrobots[1].pose_on_map_in_pix.y > (myrobots[1].goal_on_map.y - 10)) && (myrobots[1].pose_on_map_in_pix.y < (myrobots[1].goal_on_map.y + 10))){
        calibration_in_progress = false;
        ROS_INFO("CALIB GOAL completed");
      }
    }
  }
}


//одна поступившая карта и одна ее обработка это один цикл
unsigned int cycle = 0; //счетчик циклов для проверки что робот не застрял.

//проверяем что робот застрял. Если в течении 90 циклов он не отдаляется от одной точки, то он признан застрявшим
void check_if_robots_stuck(){
    //проверяем что не застрял
  if((cycle%10) == 0){
    for (int r = 0; r < ROBOT_AMOUNT; r++){
      if (((myrobots[r].pose_on_map_in_pix.x < (myrobots[r].possible_stuck_pose.x-10)) || (myrobots[r].pose_on_map_in_pix.x > (myrobots[r].possible_stuck_pose.x+10)))
                 || ((myrobots[r].pose_on_map_in_pix.y < (myrobots[r].possible_stuck_pose.y-10)) || (myrobots[r].pose_on_map_in_pix.y > (myrobots[r].possible_stuck_pose.y+10)))){
        myrobots[r].possible_stuck_pose = myrobots[r].pose_on_map_in_pix;
        myrobots[r].stuck_check = 0;
        myrobots[r].stuck = false;
      } else {
        myrobots[r].stuck_check++;
        if (myrobots[r].stuck_check>8){
          myrobots[r].stuck = true;
          //ROS_ERROR("ROBOT %d is STUCK",r)
        }
      }
      
      if (myrobots[r].stuck_check > 8){
        myrobots[r].stuck = true;
      }
    }
  }
}
























/*
|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
|||||   |||||||||   |||||||||||||||   ||||||||||||||||||          |||||||||||||||||||||||||||||||||||||||
|||||      |||      |||||||||||||       ||||||||||||||||   ||||||    ||||||||||||||||||||||||||||||||||||
|||||   |||   |||   |||||||||||   |||||   ||||||||||||||   ||||||    ||||||||||||||||||||||||||||||||||||
|||||   |||||||||   |||||||||   ||||||||   |||||||||||||           ||||||||||||||||||||||||||||||||||||||
|||||   |||||||||   ||||||||                ||||||||||||   ||||||||||||||||||||||||||||||||||||||||||||||
|||||   |||||||||   ||||||   |||||||||||||   |||||||||||   ||||||||||||||||||||||||||||||||||||||||||||||
|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
*/




//Как только ретранслятор карты запуститься, то тогда переменная получит статус false
bool map_retranslator_is_launching = true;

//ПОЛУЧЕНИЕ НОВОЙ КАРТЫ И ЕЕ ОБРАБОТКА

TickMeter timer; //создаем таймер

//ФУНКЦИЯ ПРИНИМАЕТ КАРТУ И ОБРАБАТЫВАЕТ ЕЕ
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  ROS_INFO("==========================================");
   
  
  
  timer.start();

  cycle++; //считаем циклы

  //После первого получения карты мы обозначаем что ретранслятор карты запустился
  map_retranslator_is_launching = false;

  nav_msgs::OccupancyGrid mod_merged_map = *msg;


  //если карта расширяется вбок, то увеличивается height. то есть в сторону y



  map_width = mod_merged_map.info.width;
  map_height = mod_merged_map.info.height;


  std_msgs::Header header = msg->header;
  nav_msgs::MapMetaData info = msg->info;



  ROS_INFO("Got map %d %d", info.width, info.height);

  //получаем позицию робота в пикселях начиная с правого нижнего угла карты
  get_robot_pose_in_pixels(info); 

  ROS_INFO("got robot poses");
 
  check_if_robots_stuck();

  //Наполним координаты Сейф зоны робота вокруг него, чтобы потом их использовать
  set_safe_zone_for_robots();


  //заполняем карту  пустыми пространствами под роботом
  for (int r = 0;r < ROBOT_AMOUNT; r++){
    for (int i = 0; i < 9; i++){
      mod_merged_map.data[myrobots[r].safe_zone[i].x + info.width* myrobots[r].safe_zone[i].y] = 0;
    }
  }

  


  //Перевод в формат opencv
  fill_opencv_map(msg, header, info); 
  ROS_INFO("OPENCV map filled");


  //увеличиваем стены, чтобы затмить пробелы и получить радиус робота вдоль стены
  //без этой функции стены не отмечаются в принципе
  make_walls_bigger(msg, header, info); 
  ROS_INFO("walls made");
  //находим ближайшую неизветную точку, отмечаем ее плюсом и строим к ней путь

  

  //ВЫВОД ИНФОРМАЦИ ДЛЯ ПОЛЬЗОВАТЕЛЯ
  Mat color_img; //будущая цветная карта
  Mat map_show_big; //увеличенная карта для показа

  //Переводим обычную карту в цветную
  cvtColor(map_cv, color_img, COLOR_GRAY2RGB);

  //DEBUG
  //imshow("my map", map_cv);
  //waitKey(600);


  //находим все границы
  vector<Point> borders = find_borders();
  //ROS_INFO("SIZE OF BORDERS %ld",borders.size());


  //проверяем что какие-либо роботы используют общую карту
  int global_map_users = 0;
  for (int r = 0; r < ROBOT_AMOUNT; r++){
    if(robot_use_local_map[r] == false){
      global_map_users++;
    }
  }

  if(global_map_users >= 2){

  //делаем цели из полученного массива границ
  make_targets_from_borders(borders);
  ROS_INFO("made targets from borders");

  //USER_INFO
  //найденные цели обозначаем на цветной карте
  for (int d = 0; d < all_targets.size(); d++ ){
    drawMarker(color_img, Point(all_targets[d].y, all_targets[d].x), Scalar(255, 0, 255), 0, 2);
  }
  ROS_INFO("draw all found targets");


  //для каждого робота находим путь до всех целей
  for (int r = 0; r < ROBOT_AMOUNT; r++ ){
    find_way_to_all_targets(r);
  }
  ROS_INFO("found ways to all targets");


  //выдаем цели роботам, здесь происходит выбор комбинаций
  give_targets_to_robots(); //выдаем цели роботам
  ROS_INFO("All robots had targets");

  merge_calibration();

  //ОТПРАВЛЯЕМ ЦЕЛЬ для каждого робота ему в список. Перед эти цель конвертируется в реальные координаты
  for (int r = 0; r < ROBOT_AMOUNT; r++){
    convert_goal_coord_from_map_pix_to_real(info, r);
  }
  ROS_INFO("targets are converted and send");

  //очищаем список целей
  all_targets.clear();

  //USER_INFO
  //рисуем позицию каждого робота на цветной карте
  for (int r = 0; r < ROBOT_AMOUNT; r++ ){
    drawMarker(color_img, Point(myrobots[r].pose_on_map_in_pix.y, myrobots[r].pose_on_map_in_pix.x), Scalar(0, 0, 255), r, 5);
  }

  //USER_INFO
  //увеличиваем карту, для удобного восприятия
  resize(color_img, map_show_zoom, Size(map_cv.cols*2, map_cv.rows*2), 0, 0);


  }


  //Публикация модифицированной карты с белыми пространствами под роботами
  modif_merged_map_publisher.publish(mod_merged_map);


  timer.stop();
  ROS_INFO("mapCallback took %f ms before img show", timer.getTimeMilli());
  timer.reset();

  for (int r = 0; r < ROBOT_AMOUNT; r++){
    cout << robot_use_local_map [r] << " ";

  }


  #ifdef MYDEBUG
    for (int r = 0; r < ROBOT_AMOUNT; r++ ){
      if (myrobots[r].stuck){
        cout << "Robot " << r << " is stuck!!!" << endl;
      } else {
        cout << "Robot " << r << " ok" << endl;
      }

    }
    for (int r = 0; r < ROBOT_AMOUNT; r++ ){
      
      cout << "Robot " << r << " checks = " << myrobots[r].stuck_check << endl;
      
    }
    cout << "cycle = " << cycle << endl;
  #endif //MYDEBUG


  //USER_INFO
  //Вывод цветной карты на экран
  #ifdef MYDEBUG
  //imshow("my map", map_show_zoom);
  #endif //MYDEBUG
  //imshow("my map", map_cv);
  waitKey(600);

}


/*
void mapCallback1(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  ROS_ERROR("GOt ROBOT MAP");
}
*/






























//Массив роботов, в нем хранится информация о всех роботах во время работы каждого робота на собственной карте
MyRobot mylocalrobots[ROBOT_AMOUNT];


void get_robot_pose_in_pixels_local_map (const nav_msgs::MapMetaData& info, int r){
  //Находим координаты правого нижнего угла карты относительно ее центра
  float map_corner_X = info.origin.position.x + COORD_OFFSET_LOCAL_MAP;
  float map_corner_Y = info.origin.position.y + COORD_OFFSET_LOCAL_MAP;

  //Находим положение робота относительно правого нижнего угла карты в пикселях
  
  mylocalrobots[r].pose_on_map_in_pix.x = int(20.0*(mylocalrobots[r].pose_from_center.x - map_corner_X));
  mylocalrobots[r].pose_on_map_in_pix.y = int(20.0*(mylocalrobots[r].pose_from_center.y - map_corner_Y));

}


void set_safe_zone_for_robots_local_map(int r){

  const int dx[] = {0, 0, 1, 0, -1, 1, -1, 1, -1};
  const int dy[] = {0, -1, 0, 1, 0, 1, -1, -1, 1};

  
  //очищаем массив
  for (int i = 0; i < 9; i++){
    mylocalrobots[r].safe_zone[i].x = 0;
    mylocalrobots[r].safe_zone[i].y = 0;
  }

  //заполняем массив
  for (int i = 0; i < 9; i++){
    mylocalrobots[r].safe_zone[i].x = mylocalrobots[r].pose_on_map_in_pix.x + dx[i];
    mylocalrobots[r].safe_zone[i].y = mylocalrobots[r].pose_on_map_in_pix.y + dy[i];
  }
  
}




void fill_opencv_local_map (const nav_msgs::OccupancyGrid::ConstPtr& msg, const std_msgs::Header header, const nav_msgs::MapMetaData info, Mat cvmap){
  for (unsigned int y = 0; y < info.height; y++){
    for (unsigned int x = 0; x < info.width; x++){
      
      if(msg->data[x+ info.width* y]==-1){ // -1 is unknown
      
        cvmap.at<unsigned char>(x, y) = 150;

      } else if(msg->data[x+ info.width * y]==0) { // свободно
        cvmap.at<unsigned char>(x, y) = 255;
      }
    }
  }
}



void make_walls_bigger_local_map(const nav_msgs::OccupancyGrid::ConstPtr& msg, const std_msgs::Header header, const nav_msgs::MapMetaData info, Mat cvmap, int r){
    //проходим еще раз, чтобы выставить радиусы стенам.
  bool robot_is_detected_as_wall = false;
  for (unsigned int y = 0; y < info.height; y++){
    for (unsigned int x = 0; x < info.width; x++){
      if(msg->data[x+ info.width * y]==100){ // если занято, то наносим круг

        
          for (int i = 0; i < 9; i++){
            //если координаты совпадают хоть раз с сейф зоной, то стена не наносится
            if ((x == mylocalrobots[r].safe_zone[i].x) && (y == mylocalrobots[r].safe_zone[i].y)){
              robot_is_detected_as_wall = true;
            }
          }
        
        //наносим круг в случае если робот не перекрывается стеной
        if (!robot_is_detected_as_wall){
          circle(cvmap, cv::Point(y, x), 2, Scalar::all(0), -1);
        } 
        //возвращем в исходное состояние
        robot_is_detected_as_wall = false;
      } 
    }
  }

  //нарисуем сейф зону вокруг робота

  for (int i = 0; i < 9; i++){
    cvmap.at<unsigned char>(mylocalrobots[r].safe_zone[i].x, mylocalrobots[r].safe_zone[i].y) = 255;
  }
}


//если площадь окажется мала, мы ее запишем в этот массив и закрасим
vector<Point> temp_area_local_map;

//передав неизветную точку, мы выполним волну от этой точки с заданной глубиной, вернет середину этой волны
Point make_wave_in_unknown_area_local_map(Point start, int personal_target_deep, Mat cvmap){

  ROS_ERROR("func make_wave_in_unknown_area_local_map");
  //cout << "making wave in unknown area" << endl;
  //int target_deep = global_target_deep;

  int x_size = 500;
  int y_size = 500;
  vector < vector <int> > my_cost_map(x_size, vector <int> (y_size));

  //запишем в этот массив стены, неизвестные точки
  for (int x = 0; x < x_size; x++){
    for (int y = 0; y < y_size; y++){
      if (cvmap.at<unsigned char>(x, y) == CV_UNKNOWN){
        my_cost_map[x][y] = MYCOST_UNKNOWN; //неизвестная клетка
      } else if (cvmap.at<unsigned char>(x, y) == CV_OCCUPED){
        my_cost_map[x][y] = MYCOST_OCCUPED; //стена
      } else if (cvmap.at<unsigned char>(x, y) == CV_FREE){
        my_cost_map[x][y] = MYCOST_FREE; //свободная клетка
      }
    }
  }

  //смешение относительно текущей точки
  const int dx[] = {0,1,0,-1};
  const int dy[] = {-1,0,1,0};

  int nstep = 0;
    //создаем волну 1 и волну 2. По очереди в них будем записывать координаты клеток наших волн
  vector <Point> wave_1;
  vector <Point> wave_2;
  vector < vector <Point> > waves = {wave_1,wave_2};
  int main_wave = 0;
  int second_wave = 1;
    
  my_cost_map[start.x][start.y] = 0;
  waves[main_wave].push_back(Point(start.x,start.y));
  temp_area_local_map.push_back(Point(start.x,start.y));

  //сюда будем записывать координаты, чтобы не обращаться к одним и тем же элементам векторов много раз
  int coord_x = 0;
  int coord_y = 0;

  Point possible_target;

  while(1){
    waves[second_wave].clear();
    nstep++;
    //расширяем в 4 направления точки из первой волны. Новые записываем во вторую
    for (int i = 0; i < waves[main_wave].size(); i++){
      for (int side = 0; side < 4; side++){

        //записываем координаты
        coord_x = waves[main_wave][i].x + dx[side];
        coord_y = waves[main_wave][i].y + dy[side];

        if ((coord_x >= x_size) || (coord_x <= 0)){
          continue;
        }
        if ((coord_y >= y_size) || (coord_y <= 0)){
          continue;
        }

        //если это неизвестная
        if(my_cost_map[coord_x][coord_y] == MYCOST_UNKNOWN){
         my_cost_map[coord_x][coord_y] = nstep;
         waves[second_wave].push_back(Point(coord_x, coord_y)); //заносим в волну координаты свободной точки
         temp_area_local_map.push_back(Point(coord_x, coord_y));
        } 
      }
    }
    
    
    
    if ((waves[main_wave].size() == 0) && (waves[second_wave].size() == 0)){
      //cout << "both waves = 0;" << endl; 
    //если обе волны равны нулю раньше времени, то возвращаем стартовую точку
    return start;
    }
    if (nstep > personal_target_deep){ //если мы дошли до 41 шага
    //cout << "got 40 steps sucsess" << endl;

      
      //ТАК БЫЛО ДО ВЫБОРА САМОЙ ДАЛЬНЕЙ НЕИЗВЕСТНОЙ ТОЧКИ В ВОЛНЕ
      //делим волну пополам и возвращаем середину волны
      temp_area.clear();
      return waves[second_wave][waves[second_wave].size()/2];
      
    }

    swap(main_wave, second_wave);
  }

}



//Функция находит ближайшую неизвестную область от конкретного робота
Point find_closest_big_unknown_area_local_map(Point start, Mat cvmap){ 
  ROS_ERROR("func find_closest_big_unknown_area_local_map");
  //cout << "find_closest_big_unknown_area" << endl;
  //int target_deep = global_target_deep;
  int personal_target_deep = 80; //было 40, стало 80
  int try_amount = 20; //если в течении 100 попыток не нашли нужную площадь
  int x_size = 500;
  int y_size = 500;
  vector < vector <int> > my_cost_map(x_size, vector <int> (y_size));

  //запишем в этот массив стены, неизвестные точки
  for (int x = 0; x < x_size; x++){
    for (int y = 0; y < y_size; y++){
      if (cvmap.at<unsigned char>(x, y) == CV_UNKNOWN){
        my_cost_map[x][y] = MYCOST_UNKNOWN; //неизвестная клетка
      } else if (cvmap.at<unsigned char>(x, y) == CV_OCCUPED){
        my_cost_map[x][y] = MYCOST_OCCUPED; //стена
      } else if (cvmap.at<unsigned char>(x, y) == CV_FREE){
        my_cost_map[x][y] = MYCOST_FREE; //свободная клетка
      }
    }
  }

  ROS_ERROR("map int made");

  //смешение относительно текущей точки
  const int dx[] = {0,1,0,-1};
  const int dy[] = {-1,0,1,0};

  int nstep = 0;
    //создаем волну 1 и волну 2. По очереди в них будем записывать координаты клеток наших волн
  vector <Point> wave_1;
  vector <Point> wave_2;
  vector < vector <Point> > waves = {wave_1,wave_2};
  int main_wave = 0;
  int second_wave = 1;
    
  my_cost_map[start.x][start.y] = 0;
  waves[main_wave].push_back(Point(start.x,start.y));

  //сюда будем записывать координаты, чтобы не обращаться к одним и тем же элементам векторов много раз
  int coord_x = 0;
  int coord_y = 0;

  Point possible_target;

  while(1){
    waves[second_wave].clear();
    nstep++;
    //расширяем в 4 направления точки из первой волны. Новые записываем во вторую
    for (int i = 0; i < waves[main_wave].size(); i++){
      for (int side = 0; side < 4; side++){

        //записываем координаты
        coord_x = waves[main_wave][i].x + dx[side];
        coord_y = waves[main_wave][i].y + dy[side];

        if ((coord_x >= x_size) || (coord_x <= 0)){
          continue;
        }
        if ((coord_y >= y_size) || (coord_y <= 0)){
          continue;
        }

        //если это известная
        if(my_cost_map[coord_x][coord_y] == MYCOST_FREE){
          my_cost_map[coord_x][coord_y] = nstep;
          waves[second_wave].push_back(Point(coord_x, coord_y)); //заносим в волну координаты свободной точки
        } else if (my_cost_map[coord_x][coord_y] == MYCOST_UNKNOWN){
          Point possible_target(coord_x,coord_y);
          //проверяем площадь
          Point unknown_area_target = make_wave_in_unknown_area_local_map(possible_target, personal_target_deep, cvmap);
          if ((unknown_area_target.x != possible_target.x)&& (unknown_area_target.y != possible_target.y)){
            cout << "return turget" << endl;
            return unknown_area_target;
          } else {
            //если площадь мала, то заполняем ее пустыми ячейками
            for (int c = 0; c < temp_area.size(); c++){
              my_cost_map[temp_area[c].x][temp_area[c].y] = nstep;
              waves[second_wave].push_back(Point(temp_area[c].x, temp_area[c].y));
            }
            temp_area.clear();
            /*
            //площадь мала, выставляем значение точки как nstep
            my_cost_map[coord_x][coord_y] = nstep;
            waves[second_wave].push_back(Point(coord_x, coord_y));
            */
            try_amount--;
            if (try_amount < 0){ //если в течении 20 попыток не нашли нужную по размеру область, то едем к неизв точке.
              return possible_target;
            }
            cout << "CELL [" << coord_x << "][" << coord_y << "] set as free"<< endl;
          }
        }
      }
    }
    //менем волны местами
    swap(main_wave, second_wave);
    if ((waves[main_wave].size() == 0) && (waves[second_wave].size() == 0)){ 
    //если обе волны равны нулю раньше времени
    //тогда неизвестных точек нет
    ROS_ERROR("PERSONAL TARGET NOT FOUND");
    return start;
    }
  }
}





Point find_new_personal_target_for_robot_local_map(int robot_index, Mat cvmap){
  ROS_ERROR("func find_new_personal_target_for_robot_local_map");
  Point robot_target = find_closest_big_unknown_area_local_map(mylocalrobots[robot_index].pose_on_map_in_pix, cvmap);
  return robot_target;
}

void set_personal_target_local_map(int r, Mat cvmap){
  ROS_ERROR("func set_personal_target_local_map");
  mylocalrobots[r].goal_on_map = find_new_personal_target_for_robot_local_map(r, cvmap);

}





//функция переводит координаты из пикселей в метры
void convert_goal_coord_from_map_pix_to_real_local_map(const nav_msgs::MapMetaData& info, int robot_index){
  float map_corner_X = info.origin.position.x + COORD_OFFSET_LOCAL_MAP;
  float map_corner_Y = info.origin.position.y + COORD_OFFSET_LOCAL_MAP;
  float target_x = (float)(mylocalrobots[robot_index].goal_on_map.x);
  float target_y = (float)(mylocalrobots[robot_index].goal_on_map.y);
  mylocalrobots[robot_index].goal.x = target_x/20.0 + map_corner_X;
  mylocalrobots[robot_index].goal.y = target_y/20.0 + map_corner_Y;
  mylocalrobots[robot_index].goal_detected = true;
  return;
}









void mapCallback0(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  std_msgs::Header header = msg->header;
  nav_msgs::MapMetaData info = msg->info;
  int r = 0; //номер робота
  if (robot_use_local_map[r]){


    get_robot_pose_in_pixels_local_map(info, r);

    set_safe_zone_for_robots_local_map(r);

    fill_opencv_local_map(msg, header, info, map_cv0);

    make_walls_bigger_local_map(msg, header, info, map_cv0, r);

    /*
    int free_cells = 0;
    int merged_free_cells = 0;
    for (int x = 0; x < map_cv.rows; x = x + 4){
      for (int y = 0; y < map_cv.cols; y = y + 4){
        if (map_cv0.at<unsigned char>(x,y) == CV_FREE){
          free_cells++;
          if (map_cv0.at<unsigned char>(x,y) == map_cv.at<unsigned char>(x,y)){
            merged_free_cells++;
          }
        }
      }
    }
    */
    //ROS_ERROR("%f of cells are on main map", (float)merged_free_cells/(float)free_cells);

    set_personal_target_local_map(r, map_cv0);


    Mat color_img; //будущая цветная карта

    //Переводим обычную карту в цветную
    cvtColor(map_cv0, color_img, COLOR_GRAY2RGB);

    //рисуем позицию робота на цветной карте
    
    drawMarker(color_img, Point(mylocalrobots[r].pose_on_map_in_pix.y, mylocalrobots[r].pose_on_map_in_pix.x), Scalar(0, 0, 255), r, 5);
    
    ROS_ERROR("Personal target found and set");

    drawMarker(color_img, Point(mylocalrobots[r].goal_on_map.y, mylocalrobots[r].goal_on_map.x), Scalar(255, 0, 255), 0, 2);

    convert_goal_coord_from_map_pix_to_real_local_map(info, r);

    ROS_ERROR("Personal are converted");

    //imshow("my map0", color_img);
    waitKey(600);
  }
}

void mapCallback1(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  std_msgs::Header header = msg->header;
  nav_msgs::MapMetaData info = msg->info;
  int r = 1; //номер робота
  if (robot_use_local_map[r]){

    get_robot_pose_in_pixels_local_map(info, r);

    set_safe_zone_for_robots_local_map(r);

    fill_opencv_local_map(msg, header, info, map_cv1);

    make_walls_bigger_local_map(msg, header, info, map_cv1, r);

    /*
    int free_cells = 0;
    int merged_free_cells = 0;
    for (int x = 0; x < map_cv.rows; x = x + 4){
      for (int y = 0; y < map_cv.cols; y = y + 4){
        if (map_cv1.at<unsigned char>(x,y) == CV_FREE){
          free_cells++;
          if (map_cv1.at<unsigned char>(x,y) == map_cv.at<unsigned char>(x,y)){
            merged_free_cells++;
          }
        }
      }
    }
    ROS_ERROR("%f of cells are on main map", (float)merged_free_cells/(float)free_cells);
    */

    set_personal_target_local_map(r, map_cv1);


    Mat color_img; //будущая цветная карта

    //Переводим обычную карту в цветную
    cvtColor(map_cv1, color_img, COLOR_GRAY2RGB);

    //рисуем позицию робота на цветной карте
    
    drawMarker(color_img, Point(mylocalrobots[r].pose_on_map_in_pix.y, mylocalrobots[r].pose_on_map_in_pix.x), Scalar(0, 0, 255), r, 5);
    
    ROS_ERROR("Personal target found and set");

    drawMarker(color_img, Point(mylocalrobots[r].goal_on_map.y, mylocalrobots[r].goal_on_map.x), Scalar(255, 0, 255), 0, 2);

    convert_goal_coord_from_map_pix_to_real_local_map(info, r);

    ROS_ERROR("Personal are converted");

    //imshow("my map1", color_img);
    waitKey(600);
  }
}

void mapCallback2(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  
  std_msgs::Header header = msg->header;
  nav_msgs::MapMetaData info = msg->info;
  int r = 2; //номер робота
  if (robot_use_local_map[r]){

    


    get_robot_pose_in_pixels_local_map(info, r);

    set_safe_zone_for_robots_local_map(r);

    fill_opencv_local_map(msg, header, info, map_cv2);

    make_walls_bigger_local_map(msg, header, info, map_cv2, r);

    /*
    int free_cells = 0;
    int merged_free_cells = 0;
    for (int x = 0; x < map_cv.rows; x = x + 4){
      for (int y = 0; y < map_cv.cols; y = y + 4){
        if (map_cv2.at<unsigned char>(x,y) == CV_FREE){
          free_cells++;
          if (map_cv2.at<unsigned char>(x,y) == map_cv.at<unsigned char>(x,y)){
            merged_free_cells++;
          }
        }
      }
    }
    ROS_ERROR("%f of cells are on main map", (float)merged_free_cells/(float)free_cells);
    */


    set_personal_target_local_map(r, map_cv2);


    Mat color_img; //будущая цветная карта

    //Переводим обычную карту в цветную
    cvtColor(map_cv2, color_img, COLOR_GRAY2RGB);

    //рисуем позицию робота на цветной карте
    
    drawMarker(color_img, Point(mylocalrobots[r].pose_on_map_in_pix.y, mylocalrobots[r].pose_on_map_in_pix.x), Scalar(0, 0, 255), r, 5);
    
    ROS_ERROR("Personal target found and set");

    drawMarker(color_img, Point(mylocalrobots[r].goal_on_map.y, mylocalrobots[r].goal_on_map.x), Scalar(255, 0, 255), 0, 2);

    convert_goal_coord_from_map_pix_to_real_local_map(info, r);

    ROS_ERROR("Personal are converted");

    //imshow("my map2", color_img);
    waitKey(600);
  }
}



/*
void mapCallback1(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  ROS_ERROR("MAP 1 TURTLE");
}

void mapCallback2(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  ROS_ERROR("MAP 2 TURTLE");
}

*/






/*

==============================================================================================================================
==============================================================================================================================
==============================================================================================================================
==============================================================================================================================
==============================================================================================================================
==============================================================================================================================
==============================================================================================================================
==============================================================================================================================
==============================================================================================================================
==============================================================================================================================
*/




int main(int argc, char **argv){

  //Запуск узла
  ros::init(argc, argv, "mult_task_plan_node");
  ros::NodeHandle n;

  //публикация карты ретранслятором
  modif_merged_map_publisher = n.advertise<nav_msgs::OccupancyGrid>("map", 50, true);

  //имена роботов записываем в класс
  myrobots[0].name = "tb3_0";
  myrobots[1].name = "tb3_1";
  myrobots[2].name = "tb3_2";

  //Создаем action client для каждого робота
  MoveBaseClient ac0("tb3_0/move_base", true); //для работы мув бэйз
  MoveBaseClient ac1("tb3_1/move_base", true);
  MoveBaseClient ac2("tb3_2/move_base", true);

  //Создаем массив из этих клиентов
  MoveBaseClient *ac[] = {&ac0,&ac1,&ac2};
 
  //Подписываемся на карту
  ros::Subscriber map_sub = n.subscribe("map1",2,mapCallback); //2000

  //Подписываемся на карту каждого робота в отдельности
  ros::Subscriber map_sub0 = n.subscribe("tb3_0/map",2,mapCallback0);
  ros::Subscriber map_sub1 = n.subscribe("tb3_1/map",2,mapCallback1);
  ros::Subscriber map_sub2 = n.subscribe("tb3_2/map",2,mapCallback2);
  //test
  //ros::Subscriber map_sub1 = n.subscribe("map",2000,mapCallback1);

  ROS_INFO("map was connected!");
  //подождем пока нормально начнет транслироваться карта
  
  //Запуска ретранслятор карты
  while (map_retranslator_is_launching){
  ROS_INFO("Waiting for map retranslator");
  ros::spinOnce();
  }

  //Все обращения к ac идут через точку ПРИМЕР: !ac.waitForServer, но так как мы используем указатель, то у нас ->
  for (int r = 0; r < ROBOT_AMOUNT; r++){
	  while(!ac[r]->waitForServer(ros::Duration(5.0))){ //ждем мув бэйз
		  ROS_INFO("Waiting for move_base tb3_%d",r);
	  }
    ROS_INFO("move_base tb3_%d was connected!",r);
  }
 

  tf::TransformListener listener[ROBOT_AMOUNT]; //для извлечения позиции робота на карте
 
  move_base_msgs::MoveBaseGoal goal[ROBOT_AMOUNT];// цель для мувбэйз

  for (int r = 0; r < ROBOT_AMOUNT; r++){
    goal[r].target_pose.header.frame_id = "map";
	  goal[r].target_pose.header.stamp = ros::Time::now();
  }

  while (n.ok()){ 
    ros::spinOnce();
    tf::StampedTransform transform[ROBOT_AMOUNT];
    

    for (int r = 0; r < ROBOT_AMOUNT; r++){
    try //Пробуем получить позицию робота относительно центра карты
      {
        listener[r].waitForTransform("/map", myrobots[r].name +"/base_link", ros::Time::now(), ros::Duration(1.0));

        listener[r].lookupTransform("/map", myrobots[r].name +"/base_link", ros::Time(0), transform[r]);

        myrobots[r].pose_from_center.x = transform[r].getOrigin().x();
        myrobots[r].pose_from_center.y = transform[r].getOrigin().y();
        //для локальной карты 
        mylocalrobots[r].pose_from_center.x = transform[r].getOrigin().x();
        mylocalrobots[r].pose_from_center.y = transform[r].getOrigin().y();
      }
    catch (tf::TransformException ex)
      {
        ROS_ERROR("Nope! %s", ex.what());
      }
    }


    for (int r = 0; r < ROBOT_AMOUNT; r++){
      //если появилась новая цель
      ROS_ERROR("1. going to publish turgets");
      if (robot_use_local_map[r] == false){
        ROS_ERROR("2. Robot use GLOBAL map");
        if(myrobots[r].goal_detected){
          ROS_ERROR("3. publish GLOBAL goal robot  %d",r);
          goal[r].target_pose.pose.position.x = myrobots[r].goal.x;
          goal[r].target_pose.pose.position.y = myrobots[r].goal.y;
          goal[r].target_pose.pose.orientation.w = 1.57;
          myrobots[r].goal_detected = false;
          

          ac[r]->sendGoal(goal[r]);
          
          /*
          ЭТА КОНСТРУКЦИЯ НА СЛУЧАЙ ЕСЛИ МЫ ХОДИМ ПОЛУЧАТЬ РЕЗУЛЬТАТ О ДОСТИЖЕНИИ ЦЕЛИ
          ac.waitForResult();
          
          ros::Duration(1.0).sleep();
          if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("target succeeded");
          } else {
            ROS_INFO("target failed");
          }
          */
        }
      } else {
        
        if(mylocalrobots[r].goal_detected){
          ROS_ERROR("publish LOCAL goal robot  %d",r);
          ROS_ERROR("Robot use local map");
          goal[r].target_pose.pose.position.x = mylocalrobots[r].goal.x;
          goal[r].target_pose.pose.position.y = mylocalrobots[r].goal.y;
          goal[r].target_pose.pose.orientation.w = 1.57;
          mylocalrobots[r].goal_detected = false;
          

          ac[r]->sendGoal(goal[r]);
        }
      }
    }
  }


  ros::spin();
  return 0;
}

