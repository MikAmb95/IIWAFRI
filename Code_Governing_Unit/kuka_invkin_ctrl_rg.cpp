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


#include <chrono>

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
using namespace std::chrono;
                                                                                           
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

double qdlim1 = 85*3.14/180;//85
double qdlim2 = 85*3.14/180;//85
double qdlim3 = 100*3.14/180;//100
double qdlim4 = 75*3.14/180;//75
double qdlim5 = 130*3.14/180;//130
double qdlim6 = 135*3.14/180;//135
double qdlim7 = 135*3.14/180;//135



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
		void ctrl_loop();
                VectorXd RG_function();
                VectorXd forw_dy2(VectorXd pos,VectorXd vel,VectorXd v);

	private:
      	        int _jstate_socket;
                int _jcommand_socket;

	
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
                
		ros::Publisher _cmd_pub[7];
                ros::Publisher _my_pub_q[7];
                ros::Publisher _my_pub_qd[7];
                ros::Publisher _my_pub_qr[7];

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
         _ref_q->data[2] = -15*3.14/180;
         _ref_q->data[3] = 20*3.14/180;
         _ref_q->data[4] = -15*3.14/180;
         _ref_q->data[5] = 20*3.14/180;
         _ref_q->data[6] = -30*3.14/180;

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
        Kp(0,0) = 100*0+300;
        Kp(1,1) = 100*0+300;
        Kp(2,2) = 100*0+300;
        Kp(3,3) = 300;
        Kp(4,4) = 100*0+300;
        Kp(5,5) = 100*0+300;
        Kp(6,6) = 100*0+300;


	Kd = MatrixXd::Zero(7,7);
        Kd(0,0) = 0.1;
        Kd(1,1) = 0.1;
        Kd(2,2) = 0.1;
        Kd(3,3) = 0.1;
        Kd(4,4) = 0.1;
        Kd(5,5) = 0.1;
        Kd(6,6) = 0.1;
	return true;
}


KUKA_INVKIN::KUKA_INVKIN() {

	if (!init_robot_model()) exit(1); 
	ROS_INFO("Robot tree correctly loaded from parameter server!");

	cout << "Joints and Link: " << iiwa_tree.getNrOfJoints() << " - " << iiwa_tree.getNrOfSegments() << endl;
 
	
	
	
	_cmd_pub[0] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint1_position_controller/command", 0);
	_cmd_pub[1] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint2_position_controller/command", 0);
	_cmd_pub[2] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint3_position_controller/command", 0);
	_cmd_pub[3] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint4_position_controller/command", 0);
	_cmd_pub[4] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint5_position_controller/command", 0);
	_cmd_pub[5] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint6_position_controller/command", 0);
	_cmd_pub[6] = _nh.advertise< std_msgs::Float64 > ("/lbr_iiwa/joint7_position_controller/command", 0);
        

        _my_pub_q[0] = _nh.advertise< std_msgs::Float64 > ("/q1_sim",0);
        _my_pub_q[1] = _nh.advertise< std_msgs::Float64 > ("/q2_sim",0);
        _my_pub_q[2] = _nh.advertise< std_msgs::Float64 > ("/q3_sim",0);
        _my_pub_q[3] = _nh.advertise< std_msgs::Float64 > ("/q4_sim",0);
        _my_pub_q[4] = _nh.advertise< std_msgs::Float64 > ("/q5_sim",0);
        _my_pub_q[5] = _nh.advertise< std_msgs::Float64 > ("/q6_sim",0);
        _my_pub_q[6] = _nh.advertise< std_msgs::Float64 > ("/q7_sim",0);

        _my_pub_qd[0] = _nh.advertise< std_msgs::Float64 > ("/q1_des",0);
        _my_pub_qd[1] = _nh.advertise< std_msgs::Float64 > ("/q2_des",0);
        _my_pub_qd[2] = _nh.advertise< std_msgs::Float64 > ("/q3_des",0);
        _my_pub_qd[3] = _nh.advertise< std_msgs::Float64 > ("/q4_des",0);
        _my_pub_qd[4] = _nh.advertise< std_msgs::Float64 > ("/q5_des",0);
        _my_pub_qd[5] = _nh.advertise< std_msgs::Float64 > ("/q6_des",0);
        _my_pub_qd[6] = _nh.advertise< std_msgs::Float64 > ("/q7_des",0);

        _my_pub_qr[0] = _nh.advertise< std_msgs::Float64 > ("/q1_robot",0);
        _my_pub_qr[1] = _nh.advertise< std_msgs::Float64 > ("/q2_robot",0);
        _my_pub_qr[2] = _nh.advertise< std_msgs::Float64 > ("/q3_robot",0);
        _my_pub_qr[3] = _nh.advertise< std_msgs::Float64 > ("/q4_robot",0);
        _my_pub_qr[4] = _nh.advertise< std_msgs::Float64 > ("/q5_robot",0);
        _my_pub_qr[5] = _nh.advertise< std_msgs::Float64 > ("/q6_robot",0);
        _my_pub_qr[6] = _nh.advertise< std_msgs::Float64 > ("/q7_robot",0);


	listener_socket(9030, &_jstate_socket);
        create_socket("192.170.10.146",9031,&_jcommand_socket);

}

VectorXd KUKA_INVKIN::forw_dy2(VectorXd pos,VectorXd vel,VectorXd v){
               
	
        KDL::JntArray coriol_(7);
        KDL::JntArray grav_(7);
        KDL::JntArray grav2_(7);
        KDL::JntArray *q_in2;
        KDL::JntArray *q_in3;
        KDL::JntArray *dq_in2;
        Eigen::MatrixXd m(7,7);
        Eigen::MatrixXd m2(7,7);
        KDL::JntSpaceInertiaMatrix jsim_;
        jsim_.resize(_k_chain.getNrOfJoints());
        q_in2 = new KDL::JntArray( _k_chain.getNrOfJoints() );
        q_in3 = new KDL::JntArray( _k_chain.getNrOfJoints() );
	dq_in2 = new KDL::JntArray( _k_chain.getNrOfJoints() );

	VectorXd vec(14);
        
	VectorXd L;

        double Ts = 0.001;
        double i = 0;

               

                Eigen::VectorXd e = v - pos; 
                Eigen::VectorXd de = vel; 


                for(int i = 0;i<7;i++){
                dq_in2->data[i] = vel[i];
                q_in2->data[i] = pos[i];
                q_in3->data[i] = v[i];
                }
                
               
                _dyn_param->JntToMass(*q_in2, jsim_);
                _dyn_param->JntToCoriolis(*q_in2, *dq_in2, coriol_);
                _dyn_param->JntToGravity(*q_in2, grav_);
                //_dyn_param->JntToGravity(*q_in3, grav2_);

                
                m = jsim_.data;
                m2 = m.inverse();

                
                Eigen::VectorXd tao = -Kd*de + Kp*e + grav_.data;
               
                Eigen::VectorXd qdd = m2*(- coriol_.data - grav_.data + tao); 
                
                Eigen::VectorXd qd = Ts*qdd+dq_in2->data ;
                //Eigen::VectorXd q = Ts*dq_in2->data +q_in2->data;
                Eigen::VectorXd q = Ts*qd +q_in2->data;
                 
                 
		  for (int i=0;i<7;i++){
		  vec(i) = q(i);
                  
		  }
                


		  for (int i=0;i<7;i++){
                  vec(i+7) = qd(i);
                  }
                  
                 
                 
		  


                  return vec;
}

VectorXd KUKA_INVKIN::RG_function(){


float k_l = 0;
float k_u = 1;
float k = k_u;
float eps = 0.0001; 

VectorXd v_out(7);

VectorXd pq(7);
VectorXd vq(7);
VectorXd vv(7);	

VectorXd b_t(14);
VectorXd b_t1(7);
VectorXd b_t2(7);


int iter = 100;

int x;

MatrixXd M_q = MatrixXd::Zero(1,iter);
MatrixXd M_qd = MatrixXd::Zero(1,iter);
MatrixXd M_tao = MatrixXd::Zero(1,iter);
MatrixXd M_test = MatrixXd::Ones(1,iter);

KDL::JntArray grav_2(7);
KDL::JntArray *q_in3;
q_in3 = new KDL::JntArray( _k_chain.getNrOfJoints() );
VectorXd e(7);
VectorXd tao(7);
VectorXd de(7);

while(k_u-k_l > eps){

               
                
		_v_q->data = k*_ref_q->data+(1-k)*_v0_q->data;
                _v_q->data = _v_q->data + q0->data;
                vv = _v_q->data;
                pq = _q_in->data;   
                vq = _dq_in->data;

                for(int i = 0;i<iter;i++){
                b_t = KUKA_INVKIN::forw_dy2(pq,vq,vv);
                e = vv - pq; 
                de = vq;
                
                for(int k = 0;k<7;k++){
                q_in3->data[k] = vv(k);
                }
                
                _dyn_param->JntToGravity(*q_in3, grav_2);
                tao = -Kd*de + Kp*e + grav_2.data;
              
		     for (int j=0;j<7;j++){
                  b_t1(j) = b_t(j);
                  }

                  for (int j=0;j<7;j++){
                  b_t2(j) = b_t(j+7);
                  }

                pq = b_t1;
                vq = b_t2;
                
                

               if(abs(b_t1(0))<=qlim1 && abs(b_t1(1))<=qlim2 && abs(b_t1(2))<=qlim3 && abs(b_t1(3))<=qlim4 && abs(b_t1(4))<=qlim5 && abs(b_t1(5))<=qlim6 && abs(b_t1(6))<=qlim7 ){
		 M_q(0,i) = 1;
		}
		else M_q(0,i) = 0;

                if(abs(b_t2(0))<=qdlim1 && abs(b_t2(1))<=qdlim2 && abs(b_t2(2))<=qdlim3 && abs(b_t2(3))<=qdlim4 && abs(b_t2(4))<=qdlim5 && abs(b_t2(5))<=qdlim6 && abs(b_t2(6))<=qdlim7 ){
		 M_qd(0,i) = 1;
		}
		else {
                     M_qd(0,i) = 0;
                     
                     }
                if(abs(tao(0))<=ulim1 && abs(tao(1))<=ulim2 && abs(tao(2))<=ulim3 && abs(tao(3))<=ulim4 && abs(tao(4))<=ulim5 && abs(tao(5))<=ulim6 && abs(tao(6))<=ulim7 ){
		 M_tao(0,i) = 1;
		}
		else {
                     M_tao(0,i) = 0;
                     
                     }

               }
                
                
                if (M_q == M_test && M_qd == M_test && M_tao == M_test){
			         cout<<"Constraints Ok"<<endl;
				 k_l = k;
		  
		  }
		  else {
		       k_u = k;
		  }
                  k = 0.5*(k_u + k_l);
                  


}


k = k_l;

_v_q->data = k*_ref_q->data+(1-k)*_v0_q->data;
_v0_q->data = _v_q->data;	
v_out = _v_q->data;

return v_out;
}


void KUKA_INVKIN::ctrl_loop() {

std_msgs::Float64 d;
int slen2, rlen2;	
ROBOT_STATE rs2;
sockaddr_in si_other2;
float i_cmd[7];
std_msgs::Float64 cmd[7];
VectorXd v(7);
        
        
VectorXd t1(7);
VectorXd t2(7);

VectorXd vv(7);
VectorXd pq(7);
VectorXd vq(7);

int x;
int count = 0;
double param_sim[2];


ofstream myfile_qd;
ofstream myfile_p;
myfile_qd.open("/home/utente/ros_ws/src/iiwa_kdl/src/rg.m");
myfile_p.open("/home/utente/ros_ws/src/iiwa_kdl/src/prg.m");
myfile_qd<<"q_rg = [";
myfile_p<<"param_rg = [";
//while(!first_flag ) usleep(0.005);
   
       ros::Rate r(5);
       while( ros::ok() ) {	
                cout<<"CONTROL"<<endl;
                
                auto start = high_resolution_clock::now();        
                v = KUKA_INVKIN::RG_function();
                auto stop = high_resolution_clock::now();
                auto duration = duration_cast<microseconds>(stop - start);
                param_sim[0] = duration.count()*0.000001;
                cout<<"time: "<<param_sim[0]<<endl;
                count = count +1;
                param_sim[1] = count;
                cout<<"Iteration n: "<<count<<endl;

                
                _v1_q->data = v;
                vv = v + q0->data;
                
                
                ;

               if(v(0) < (29.9*3.14/180)){
                myfile_qd<<vv.transpose()<<" ";
                myfile_qd<<"\n";
                myfile_p<<param_sim[0]<<" "<<param_sim[1]<<" ";
                myfile_p<<"\n";
                }
          
                else{
                cout<<"-------------------- END ---------------- "<<endl;
                myfile_qd<<vv.transpose()<<"]; \n";
                myfile_qd.close();
                myfile_p<<param_sim[0]<<" "<<param_sim[1]<<"]; \n";
                myfile_p.close();
                }

                pq = _q_in->data;
                vq = _dq_in->data;
               
                cout<<"REF: "<<vv.transpose()<<endl;
                cout<<"POS: "<<pq.transpose()<<endl;


                VectorXd t = KUKA_INVKIN::forw_dy2(pq,vq,vv);

		for (int i=0;i<7;i++){
                  t1(i) = t(i);
                  }

                  for (int i=0;i<7;i++){
                  t2(i) = t(i+7);
                  }
                 
               
               for(int i=0; i<7; i++) {
			cmd[i].data = v(i);
		}
		for(int i=0; i<7; i++) {
			_my_pub_qd[i].publish (cmd[i]);
		}
                
         
                _dq_in->data = t2;
                _q_in->data = t1;
                
                
                r.sleep();
                
               
	}


}

void KUKA_INVKIN::run() {
	boost::thread ctrl_loop_t ( &KUKA_INVKIN::ctrl_loop, this);
	ros::spin();	
}




int main(int argc, char** argv) {
        int x;
        cout<<"RG CONTROL PRESS 1 TO START"<<endl;
        cin>>x;        
	ros::init(argc, argv, "iiwa_kdl_rg");
	KUKA_INVKIN ik;
	ik.run();

	return 0;
}
