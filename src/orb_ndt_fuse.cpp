#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "tf/message_filter.h"
#include <map>
#include <vector>

#define OFFSET4SCALE 100

std::map<int, std::pair<tf::Transform, tf::Transform> > key_map;
std::vector<std::pair<int, tf::Transform> > orb_results;

void load_keymap(char *filename){
  FILE *fp;
  int rkid,kid;
  double ts,xo,yo,zo,qxo,qyo,qzo,qwo;
  double xd,yd,zd,qxd,qyd,qzd,qwd;

  fp=fopen(filename,"r");
  if(!fp)return;

  while(fscanf(fp,"%d\t%d\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf",&rkid,&kid,&ts,&xo,&yo,&zo,&qxo,&qyo,&qzo,&qwo,&xd,&yd,&zd,&qxd,&qyd,&qzd,&qwd)!=EOF){
    if(rkid>0&&rkid<10000){
      key_map[rkid].first.setOrigin(tf::Vector3(xo,yo,zo));
      key_map[rkid].first.setRotation(tf::Quaternion(qxo,qyo,qzo,qwo));
      key_map[rkid].second.setOrigin(tf::Vector3(xd,yd,zd));
      key_map[rkid].second.setRotation(tf::Quaternion(qxd,qyd,qzd,qwd));
      /*      printf("%d %f %f\n",
	     rkid,
	     key_map[rkid].first.getOrigin().x(),
	     key_map[rkid].second.getOrigin().x()
	     );*/
    }
  }
}

void load_orb(char *filename){
  FILE *fp;
  int rkid;
  double xo,yo,zo,qxo,qyo,qzo,qwo;
 
  fp=fopen(filename,"r");
  if(!fp)return;

  while(fscanf(fp,"%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%d",
	       &xo,&yo,&zo,&qxo,&qyo,&qzo,&qwo,&rkid)!=EOF){
    while(fgetc(fp)!='\n');
    
    if(rkid>0&&rkid<10000){
      std::pair<int, tf::Transform> orb_result;
      orb_result.first=rkid;
      orb_result.second.setOrigin(tf::Vector3(xo,yo,zo));
      orb_result.second.setRotation(tf::Quaternion(qxo,qyo,qzo,qwo));
      orb_results.push_back(orb_result);

   }
  }
}

int main(int argc, char *argv[]){
  ros::init(argc, argv, "orb");
  ros::NodeHandle n;
  


  load_keymap(argv[1]);
  load_orb(argv[2]);


  for(std::vector<std::pair<int, tf::Transform> >::iterator it=orb_results.begin(); it!=orb_results.end(); it++){
    //printf("%d %f\n",it->first,it->second.getOrigin().x()); 
    if(it->first>OFFSET4SCALE){
      double scale =
	sqrt((key_map[it->first].second.getOrigin().x() -
	      key_map[it->first-OFFSET4SCALE].second.getOrigin().x())*
	     (key_map[it->first].second.getOrigin().x() -
	      key_map[it->first-OFFSET4SCALE].second.getOrigin().x())+
	     (key_map[it->first].second.getOrigin().y() -
	      key_map[it->first-OFFSET4SCALE].second.getOrigin().y())*
	     (key_map[it->first].second.getOrigin().y() -
	      key_map[it->first-OFFSET4SCALE].second.getOrigin().y())+
	     (key_map[it->first].second.getOrigin().z() -
	      key_map[it->first-OFFSET4SCALE].second.getOrigin().z())*
	     (key_map[it->first].second.getOrigin().z() -
	      key_map[it->first-OFFSET4SCALE].second.getOrigin().z())
	     )/
	sqrt((key_map[it->first].first.getOrigin().x() -
	      key_map[it->first-OFFSET4SCALE].first.getOrigin().x())*
	     (key_map[it->first].first.getOrigin().x() -
	      key_map[it->first-OFFSET4SCALE].first.getOrigin().x())+
	     (key_map[it->first].first.getOrigin().y() -
	      key_map[it->first-OFFSET4SCALE].first.getOrigin().y())*
	     (key_map[it->first].first.getOrigin().y() -
	      key_map[it->first-OFFSET4SCALE].first.getOrigin().y())+
	     (key_map[it->first].first.getOrigin().z() -
	      key_map[it->first-OFFSET4SCALE].first.getOrigin().z())*
	     (key_map[it->first].first.getOrigin().z() -
	      key_map[it->first-OFFSET4SCALE].first.getOrigin().z())
	     );
      tf::Transform orb_rel = key_map[it->first].first.inverse() * it->second;
     
      tf::Transform scaled_rel=orb_rel;
      scaled_rel.setOrigin(orb_rel.getOrigin()*scale);

      tf::Transform position=key_map[it->first].second*scaled_rel;
      
      printf("%f %f %f %f %f %f %f %f\n",scale,orb_rel.getOrigin().x(),orb_rel.getOrigin().y(),orb_rel.getOrigin().z(),position.getOrigin().x(),position.getOrigin().y(),position.getOrigin().z()
	     );
    }
  }
  return(0);
}
