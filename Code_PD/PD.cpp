#include "ros/ros.h"
#include "boost/thread.hpp"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include <std_msgs/Float64.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/mman.h>
#include <sys/io.h>
#include <sys/time.h>
#include <netdb.h>


#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chaindynparam.hpp>
#include <ros/package.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Dense>


using namespace Eigen;
using namespace std;
using namespace KDL;

int row = 30001;
int n = row;
int col = 7;
float vett[30001][7];
float q1[30001];
float q2[30001];
float q3[30001];
float q4[30001];
float q5[30001];
float q6[30001];
float q7[30001];                                                                                                                                 
                                                                                                                                
int ulim1 = 320;
int ulim2 = 320;
int ulim3 = 176;
int ulim4 = 176;
int ulim5 = 110;
int ulim6 = 40;
int ulim7 = 40;

double qlim1 = 170*3.14/180;
double qlim2 = 120*3.14/180;
double qlim3 = 170*3.14/180;
double qlim4 = 120*3.14/180;
double qlim5 = 170*3.14/180;
double qlim6 = 120*3.14/180;
double qlim7 = 175*3.14/180;

double qdlim1 = 70*3.14/180;//85
double qdlim2 = 60*3.14/180;//85
double qdlim3 = 80*3.14/180;//100
double qdlim4 = 60*3.14/180;//75
double qdlim5 = 100*3.14/180;//130
double qdlim6 = 100*3.14/180;//135
double qdlim7 = 100*3.14/180;//135



typedef struct ROBOT_STATE {
  double jstate[7];
}ROBOT_STATE;

typedef struct ROBOT_CMD {
  double jcmd[7];
}ROBOT_CMD;

//Creazione socket in LETTURA
inline bool listener_socket(int port_number, int *sock) {
      sockaddr_in si_me;

  if ( (*sock=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
    std::cout << "Listener::Open: error during socket creation!" << std::endl;
    return false;
  }

  memset((char *) &si_me, 0, sizeof(si_me));

  /* allow connections to any address port */
  si_me.sin_family = AF_INET;
  si_me.sin_port = htons(port_number);
  si_me.sin_addr.s_addr = htonl(INADDR_ANY);
  int bind_ok = bind(*sock, (struct sockaddr*)&si_me, sizeof(si_me));

  if ( bind_ok == -1 )
    return false;
  else
    return true;

}

//Creazione socket in SCRITTURA
inline int create_socket(char* dest, int port, int *sock) {
  struct sockaddr_in temp;
  struct hostent *h;
  int error;

  temp.sin_family=AF_INET;
  temp.sin_port=htons(port);
  h=gethostbyname(dest);

  if (h==0) {
    printf("Gethostbyname fallito\n");
    exit(1);
  }

  bcopy(h->h_addr,&temp.sin_addr,h->h_length);
  *sock=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  error=connect(*sock, (struct sockaddr*) &temp, sizeof(temp));
  return error;
}




class KUKA_INVKIN {
	public:
		KUKA_INVKIN();
		void run();
		bool init_robot_model();
                void pd_g();


                void joint_states_cb(std_msgs::Float64 js);
                void joint_states_cb1(std_msgs::Float64 js);
                void joint_states_cb2(std_msgs::Float64 js);
                void joint_states_cb3(std_msgs::Float64 js);
                void joint_states_cb4(std_msgs::Float64 js);
                void joint_states_cb5(std_msgs::Float64 js);
                void joint_states_cb6(std_msgs::Float64 js);


		void ctrl_loop();
                VectorXd RG_function();
                VectorXd forw_dy2(VectorXd pos,VectorXd vel,VectorXd v);

	private:
      	        int _jstate_socket;
                int _jcommand_socket;

	        
                ros::Subscriber _js_sub;
                ros::Subscriber _js_sub1;
                ros::Subscriber _js_sub2;
                ros::Subscriber _js_sub3;
                ros::Subscriber _js_sub4;
                ros::Subscriber _js_sub5;
                ros::Subscriber _js_sub6;
 
		ros::NodeHandle _nh;
		KDL::Tree iiwa_tree;
	
		KDL::ChainFkSolverPos_recursive *_fksolver; //Forward position solver	
		KDL::ChainIkSolverVel_pinv *_ik_solver_vel;   	//Inverse velocity solver
		KDL::ChainIkSolverPos_NR *_ik_solver_pos;

		KDL::Chain _k_chain;

                Eigen::MatrixXd Kd; 	
		Eigen::MatrixXd Kp;
	
		KDL::JntArray *_v_q;
		KDL::JntArray *_v0_q;
                KDL::JntArray *_v1_q;
                KDL::JntArray *q0;
		KDL::JntArray *_q_in;
                KDL::JntArray *_dq_in;
                KDL::JntArray *_ref_q;
                

                KDL::ChainDynParam *_dyn_param;

                bool first_flag;

	
};


bool KUKA_INVKIN::init_robot_model() {
	std::string robot_desc_string;
	_nh.param("robot_description", robot_desc_string, std::string());
	if (!kdl_parser::treeFromString(robot_desc_string, iiwa_tree)){
		ROS_ERROR("Failed to construct kdl tree");
		return false;
	}

	std::string base_link = "lbr_iiwa_link_0";
	std::string tip_link  = "lbr_iiwa_link_7";
	if ( !iiwa_tree.getChain(base_link, tip_link, _k_chain) ) return false;
	

	_ik_solver_vel = new KDL::ChainIkSolverVel_pinv( _k_chain );
	_ik_solver_pos = new KDL::ChainIkSolverPos_NR( _k_chain, *_fksolver, *_ik_solver_vel, 100, 1e-6 );

	_q_in = new KDL::JntArray( _k_chain.getNrOfJoints() );
	_dq_in = new KDL::JntArray( _k_chain.getNrOfJoints() );
	_ref_q = new KDL::JntArray( _k_chain.getNrOfJoints() );
        _v0_q = new KDL::JntArray( _k_chain.getNrOfJoints() );
        _v1_q = new KDL::JntArray( _k_chain.getNrOfJoints() );
        q0 = new KDL::JntArray( _k_chain.getNrOfJoints() );
        _v_q = new KDL::JntArray( _k_chain.getNrOfJoints() );
        _dyn_param = new KDL::ChainDynParam(_k_chain,KDL::Vector(0,0,-9.81));

        
        first_flag = false; 

         _q_in->data[0] = 0*3.14/180;
	 _q_in->data[1] = 0;
	 _q_in->data[2] = 90*3.14/180;
	 _q_in->data[3] = 90*3.14/180;
	 _q_in->data[4] = 0;
	 _q_in->data[5] = -90*3.14/180;
	 _q_in->data[6] = 0;

         _ref_q->data[0] = 30*3.14/180;
         _ref_q->data[1] = 20*3.14/180;
         _ref_q->data[2] = 0*10*3.14/180;
         _ref_q->data[3] = 20*3.14/180;
         _ref_q->data[4] = -15*3.14/180;
         _ref_q->data[5] = 0*10*3.14/180;
         _ref_q->data[6] = 0*3.14/180;

         q0->data[0] = 0;
         q0->data[1] = 0;
         q0->data[2] = 90*3.14/180;
         q0->data[3] = 90*3.14/180;
         q0->data[4] = 0;
         q0->data[5] = -90*3.14/180;
         q0->data[6] = 0;

	 
         _dq_in->data[0] = 0;
         _dq_in->data[1] = 0;
         _dq_in->data[2] = 0;
         _dq_in->data[3] = 0;
         _dq_in->data[4] = 0;
         _dq_in->data[5] = 0;
         _dq_in->data[6] = 0;
         
         
         _v0_q->data = _dq_in->data;
         _v1_q->data = _dq_in->data;
         _v_q->data = _dq_in->data;

                 Kp = MatrixXd::Zero(7,7);
        Kp(0,0) = 100*0+500;
        Kp(1,1) = 100*0+500;
        Kp(2,2) = 100*0+500;
        Kp(3,3) = 500;
        Kp(4,4) = 100*0+300;
        Kp(5,5) = 100*0+300;
        Kp(6,6) = 100*0+300;


	Kd = MatrixXd::Zero(7,7);
        Kd(0,0) = 2;
        Kd(1,1) = 2;
        Kd(2,2) = 2;
        Kd(3,3) = 2;
        Kd(4,4) = 2;
        Kd(5,5) = 2;
        Kd(6,6) = 2;
	return true;
}


//SUBSCRIBER

void KUKA_INVKIN::joint_states_cb( std_msgs::Float64 js ) {

	_dq_in->data[0] = js.data;
        cout<<" 1: "<<_dq_in->data[0]<<endl;
        
}

void KUKA_INVKIN::joint_states_cb1( std_msgs::Float64 js ) {

	_dq_in->data[1] = js.data;
       cout<<" 2: "<<_dq_in->data[1]<<endl;
}

void KUKA_INVKIN::joint_states_cb2( std_msgs::Float64 js ) {

	_dq_in->data[2] = js.data;
        cout<<" 3: "<<_dq_in->data[2]<<endl;
}

void KUKA_INVKIN::joint_states_cb3( std_msgs::Float64 js ) {

	_dq_in->data[3] = js.data;
        cout<<" 4: "<<_dq_in->data[3]<<endl;
        
}

void KUKA_INVKIN::joint_states_cb4( std_msgs::Float64 js ) {

	_dq_in->data[4] = js.data;
        cout<<" 5: "<<_dq_in->data[4]<<endl;
}

void KUKA_INVKIN::joint_states_cb5( std_msgs::Float64 js ) {

	_dq_in->data[5] = js.data;
       cout<<" 6: "<<_dq_in->data[5]<<endl;
}

void KUKA_INVKIN::joint_states_cb6( std_msgs::Float64 js ) {

	_dq_in->data[6] = js.data;
        cout<<" 7: "<<_dq_in->data[6]<<endl;
        }


// CODE FROM HERE

KUKA_INVKIN::KUKA_INVKIN() {

	if (!init_robot_model()) exit(1); 
	ROS_INFO("Robot tree correctly loaded from parameter server!");

        _js_sub = _nh.subscribe("/q1_des", 0, &KUKA_INVKIN::joint_states_cb, this);
        _js_sub1 = _nh.subscribe("/q2_des", 0, &KUKA_INVKIN::joint_states_cb1, this);
        _js_sub2 = _nh.subscribe("/q3_des", 0, &KUKA_INVKIN::joint_states_cb2, this);
        _js_sub3 = _nh.subscribe("/q4_des", 0, &KUKA_INVKIN::joint_states_cb3, this);
        _js_sub4 = _nh.subscribe("/q5_des", 0, &KUKA_INVKIN::joint_states_cb4, this);
        _js_sub5 = _nh.subscribe("/q6_des", 0, &KUKA_INVKIN::joint_states_cb5, this);
        _js_sub6 = _nh.subscribe("/q7_des", 0, &KUKA_INVKIN::joint_states_cb6, this);



	cout << "Joints and Link: " << iiwa_tree.getNrOfJoints() << " - " << iiwa_tree.getNrOfSegments() << endl;

	listener_socket(9030, &_jstate_socket);
        create_socket("192.170.10.146",9031,&_jcommand_socket);

}


void KUKA_INVKIN::pd_g() {

int slen2, rlen2;	
ROBOT_STATE rs2;
sockaddr_in si_other2;
ROBOT_CMD rc;
KDL::JntArray *q_read;
VectorXd q_in(7);
VectorXd q_0(7);
VectorXd q_dot(7);

VectorXd vv(7);
VectorXd pq(7);
VectorXd vq(7);

int j = 0;


rlen2 = recvfrom( _jstate_socket, &rs2, sizeof(rs2),0,(struct sockaddr*)&si_other2, (socklen_t*)&slen2);
          if(rlen2>0) {
          for(int i=0; i<7; i++) {
          q_0(i) = rs2.jstate[i];
          
}
        }



KDL::JntArray grav_(7);
q_read = new KDL::JntArray( _k_chain.getNrOfJoints() );
bool first_r = 0;


        ofstream myfile_qd;
        ofstream myfile_q;
        myfile_qd.open("/home/utente/ros_ws/src/iiwa_kdl/src/posd.m");
        myfile_q.open("/home/utente/ros_ws/src/iiwa_kdl/src/pos.m");
 
        myfile_q<<"q = [";
        myfile_qd<<"qd = [";
      
        



int count = 0;

ros::Rate r(1000);
       while( ros::ok() ) {
       //cout<<"PD_g"<<endl;


      if(first_r == 0){
      rlen2 = recvfrom( _jstate_socket, &rs2, sizeof(rs2),0,(struct sockaddr*)&si_other2, (socklen_t*)&slen2);
      for(int i=0; i<7; i++) {
      q0->data[i] = rs2.jstate[i];
      q_0(i) = rs2.jstate[i];
      }
      first_r = 1;
      }

       
       rlen2 = recvfrom( _jstate_socket, &rs2, sizeof(rs2),0,(struct sockaddr*)&si_other2, (socklen_t*)&slen2);
          if(rlen2>0) {
                 for(int i=0; i<7; i++) 
                 q_in(i) = rs2.jstate[i];
        }
        
        q_dot = (q_in - q_0)*1000;
        q_0 = q_in;
        
       cout<<"POS2: "<<q_in.transpose()<<endl;
       cout<<"REF2: ";



       for(int i = 0;i<7;i++){
       cout<<_dq_in->data[i]<<" ";
       }
       cout<<endl;
       for(int i = 0;i<7;i++){
       _v1_q->data[i] = _dq_in->data[i];
       }

        
        pq = q_in;   
        vq = _v1_q->data + q0->data;
        
        cout<<"COUNT: "<<count<<endl;
        if(count <= 30000) {
        myfile_q<<pq.transpose()<<" ";
        myfile_q<<"\n";
        myfile_qd<<vq.transpose()<<" ";
        myfile_qd<<"\n"; 
        }
        else {
        myfile_q<<pq.transpose()<<"]; \n";
        myfile_q.close();

        myfile_qd<<vq.transpose()<<"]; \n";
        myfile_qd.close();
                 
        cout<<"FINE FILE"<<endl;
        }
        count = count + 1;


        //Eigen::VectorXd e =  _ref_q->data - q_in + q0->data;
        Eigen::VectorXd e = _v1_q->data - q_in + q0->data; 
        //cout<<"ERR: "<<e.transpose()<<endl;
        Eigen::VectorXd de = q_dot; 
        
     
        Eigen::VectorXd tao = -Kd*de + Kp*e;
        
        for (int i=0; i<7; i++){
                rc.jcmd[i] = tao(i);
                
                }
                
                
        write( _jcommand_socket, &rc, sizeof(rc) ); //write commands over socket
        
        r.sleep();
        first_flag = true;
        }




}


void KUKA_INVKIN::run() {

	boost::thread pd_g ( &KUKA_INVKIN::pd_g, this);
	
	ros::spin();	

}




int main(int argc, char** argv) {
      
        
	ros::init(argc, argv, "iiwa_kdl_ctrl");
	KUKA_INVKIN ik;
	ik.run();

	return 0;
}
