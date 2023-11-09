#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv) {   

    ros::init(argc, argv, "pub_node"); //инициализация ноды
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
    ros::Rate loop_rate(1); //отправляем одно сообщение в секунду      
     
     for (int t = 0; t < 20; t++){  
     	geometry_msgs::Twist pos;  //создаем экземпляр одного сообщения
     	
     	
     	pos.linear.x=0.1*t;
     	pos.angular.z=1 - 1/20*t;
     	
     	ROS_INFO("I like to move it move it");
     	
     	pub.publish(pos); //публикуем сообщение
     	
     	loop_rate.sleep(); // ждем чтобы сохранить скорость публикации 1 сообщение в секунду
     	
     	}     

     ros::spinOnce(); //ожидание выполнения всех процессов (в данном случае публикации)

     return 0;

}  
