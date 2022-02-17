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

double qlim[7] = {170*3.14/180,120*3.14/180,170*3.14/180,120*3.14/180,170*3.14/180,120*3.14/180,175*3.14/180};


double qdlim[7] = {85*3.14/180,85*3.14/180,100*3.14/180,75*3.14/180,130*3.14/180,135*3.14/180,135*3.14/180};

double ulim[7] = {320,320,170,170,110,40,40};

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
                double ERG_function();
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
        int x = 0;
	VectorXd L;

        double Ts = 0.001;
        double i = 0;

               

                Eigen::VectorXd e = v - pos; 
                Eigen::VectorXd de = vel; 


                for(int i = 0;i<7;i++){
                dq_in2->data[i] = vel[i];
                q_in2->data[i] = pos[i];
                //q_in3->data[i] = v[i];
                }
                
               
                _dyn_param->JntToMass(*q_in2, jsim_);
                _dyn_param->JntToCoriolis(*q_in2, *dq_in2, coriol_);
                _dyn_param->JntToGravity(*q_in2, grav_);
                //_dyn_param->JntToGravity(*q_in3, grav2_);

                
                m = jsim_.data;
                m2 = m.inverse();

                
                Eigen::VectorXd tao = -Kd*de + Kp*e + grav_.data;

                //cout<<"TRQ: "<<tao.transpose()<<endl;
              
              
               /*
                if(tao(0)>=ulim1) {
			tao(0) = ulim1;
		}
		else if(tao(0)<=-ulim1) {
			tao(0) = -ulim1;
		}
		

		  if(tao(1)>=ulim2) {
                        tao(1) = ulim2;
                }
                else if(tao(1)<=-ulim2) {
                        tao(1) = -ulim2;
                }

		  if(tao(2)>=ulim3) {
                        tao(2) = ulim3;
                }
                else if(tao(2)<=-ulim3) {
                        tao(2) = -ulim3;
                }

		  if(tao(3)>=ulim4) {
                        tao(3) = ulim4;
                }
                else if(tao(3)<=-ulim4) {
                        tao(3) = -ulim4;
                }

		  if(tao(4)>=ulim5) {
                        tao(4) = ulim5;
                }
                else if(tao(4)<=-ulim5) {
                        tao(4) = -ulim5;
                }

		  if(tao(5)>=ulim6) {
                        tao(5) = ulim6;
                }
                else if(tao(5)<=-ulim6) {
                        tao(5) = -ulim6;
                }

		  if(tao(6)>=ulim7) {
                        tao(6) = ulim7;
                }
                else if(tao(6)<=-ulim7) {
                        tao(6) = -ulim7;
                }

                */
                
                
                
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

double KUKA_INVKIN::ERG_function(){


float k_l = 0;
float k_u = 1;
float k = k_u;

VectorXd v_out(7);

VectorXd pq(7);
VectorXd vq(7);
VectorXd vv(7);	

VectorXd b_t(14);
VectorXd b_t1(7);
VectorXd b_t2(7);


int iter = 100;

int x;



KDL::JntArray grav_2(7);
KDL::JntArray *q_in3;
KDL::JntSpaceInertiaMatrix jsim2_;
jsim2_.resize(_k_chain.getNrOfJoints());
q_in3 = new KDL::JntArray( _k_chain.getNrOfJoints() );
MatrixXd B(7,7);


// ERG PARAMETERS
double kq = 8;
double kqd = 5;
double ktao = 0.03;
double k_e = 85;
double E_term = 150;

                vv = _v_q->data + q0->data;
                pq = _q_in->data;   
                vq = _dq_in->data;
                //cout<<"REF: "<<vv.transpose()<<endl;
                //cout<<"POS: "<<pq.transpose()<<endl;
                //cout<<"VQ: "<<vq.transpose()<<endl;
                 
               // Dynamic Simulation // Dynamic Safety Margin

                VectorXd Delta_v2(iter);
                VectorXd Delta_v4(iter);
                VectorXd Delta_v6(iter);
                VectorXd tao(iter);
                VectorXd e(7);
                VectorXd de(7);


                for(int i = 0;i<iter;i++){
                b_t = KUKA_INVKIN::forw_dy2(pq,vq,vv);
                e = vv - pq; 
                de = vq;
               
		     for (int j=0;j<7;j++){
                  b_t1(j) = b_t(j);
                  }

                  for (int j=0;j<7;j++){
                  b_t2(j) = b_t(j+7);
                  }
                pq = b_t1;
                vq = b_t2;
                //cout<<"POS: "<<b_t1.transpose()<<endl;
                //cout<<"VQ: "<<b_t2.transpose()<<endl;
                for(int k = 0;k<7;k++){
                q_in3->data[k] = vv(k);
                }
               
                _dyn_param->JntToGravity(*q_in3, grav_2);
                tao = -Kd*de + Kp*e + grav_2.data;
                //cout<<"TAO: "<<tao.transpose()<<endl;
                VectorXd Delta_v1(7);
                for(int k = 0; k<7;k++){
                VectorXd Temp3(2);
                Temp3(0) = b_t1(k) + qlim[k];
                Temp3(1) = - b_t1(k) + qlim[k];
                Delta_v1(k) = Temp3.minCoeff();
                }
                Delta_v2(i) = Delta_v1.minCoeff();
                
                VectorXd Delta_v3(7);
                for(int k = 0; k<7;k++){
                VectorXd Temp3(2);
                Temp3(0) = b_t2(k) + qdlim[k];
                Temp3(1) = - b_t2(k) + qdlim[k];
                Delta_v3(k) = Temp3.minCoeff();
                }
                Delta_v4(i) = Delta_v3.minCoeff();
               
                VectorXd Delta_v5(7);
                for(int k = 0; k<7;k++){
                VectorXd Temp3(2);
                Temp3(0) = tao(k) + ulim[k];
                Temp3(1) = - tao(k) + ulim[k];
                Delta_v5(k) = Temp3.minCoeff();
                }
                Delta_v6(i) = Delta_v5.minCoeff();
               }
               //cin>>x;
               VectorXd Delta_vec(3);
               Delta_vec(0) =  kq*Delta_v2.minCoeff();
               Delta_vec(1) =  kqd*Delta_v4.minCoeff(); 
               Delta_vec(2) =  ktao*Delta_v6.minCoeff();            
               double Delta1 = Delta_vec.minCoeff();
               cout<<"DELTA: "<<Delta_vec.transpose()<<endl;
               for(int k = 0;k<7;k++){
                q_in3->data[k] = pq(k);
                }
               _dyn_param->JntToMass(*q_in3, jsim2_);
               B = jsim2_.data;

               VectorXd V = 0.5*b_t2.transpose()*B*b_t2 + 0.5*e.transpose()*Kp*e;
               double Delta_V = k_e*(E_term - V(0));  
               
               VectorXd Delta_vec2(2);
               Delta_vec2(0) = Delta1;
               Delta_vec2(1) = Delta_V;
               
              VectorXd Delta_vec3(2);
              Delta_vec3(0) = Delta_vec2.minCoeff();
              Delta_vec3(1) = 0;
              
              double Delta = Delta_vec3.maxCoeff();
              return Delta;
}


void KUKA_INVKIN::ctrl_loop() {

std_msgs::Float64 d;
int slen2, rlen2;	
ROBOT_STATE rs2;
sockaddr_in si_other2;
float i_cmd[7];
std_msgs::Float64 cmd[7];
VectorXd v(7);
double Delta;
int x;
        
VectorXd t1(7);
VectorXd t2(7);

VectorXd vv(7);
VectorXd pq(7);
VectorXd vq(7);
double eta = 0.00001; 
double zeta = 0.15;
double delta = 0.1;

int count = 0;
double param_sim[2];


ofstream myfile_qd;
ofstream myfile_p;
myfile_qd.open("/home/utente/ros_ws/src/iiwa_kdl/src/erg.m");
myfile_p.open("/home/utente/ros_ws/src/iiwa_kdl/src/perg.m");
myfile_qd<<"q_erg = [";
myfile_p<<"param_erg = [";


       ros::Rate r(10);
       while( ros::ok() ) {	
                cout<<"CONTROL"<<endl;
                
                auto start = high_resolution_clock::now();        
               //Navigation Field Evaluation
                VectorXd ro_att = _ref_q->data - _v_q->data;
                //cout<<"NF: "<<ro_att.transpose()<<endl;
                double ro1_att = ro_att.norm();
                //cout<<"NF: "<<ro1_att<<endl;
                VectorXd ro2_att(2);
                ro2_att(0) = ro1_att;
                ro2_att(1) = eta; 
                VectorXd ro3_att = ro_att/ro2_att.maxCoeff(); 
                //cout<<"NF: "<<ro3_att.transpose()<<endl;
                VectorXd ro_rep(7);
                for(int i=0;i<7;i++){

                double temp1 = abs(_ref_q->data[i]+qlim[i]);
                temp1 = (zeta - temp1)/(zeta - delta);
                VectorXd Temp1(2);
                Temp1(0) = temp1;
                Temp1(1) = 0;

                double temp2 = abs(_ref_q->data[i]-qlim[i]);
                temp2 = (zeta - temp2)/(zeta - delta);
                VectorXd Temp2(2);
                Temp2(0) = temp2;
                Temp2(1) = 0;

                ro_rep(i) = Temp1.maxCoeff() - Temp2.maxCoeff();

                }
                //cout<<"NF: "<<ro_rep.transpose()<<endl;
                VectorXd ro = ro3_att + ro_rep;


                
                Delta = KUKA_INVKIN::ERG_function();
                
       
                cout<<"NF: "<<ro.transpose()<<" / "<<"DSM: "<<Delta<<endl;
                VectorXd v_app = ro*Delta*0.001 + _v_q->data;
                
                
                auto stop = high_resolution_clock::now();
                auto duration = duration_cast<microseconds>(stop - start);
                param_sim[0] = duration.count()*0.000001;
                cout<<"time: "<<param_sim[0]<<endl;
                //cout<<"time: "<<duration.count()*0.000001<<endl;
                
                count = count +1;
                param_sim[1] = count;
                cout<<"Iteration n: "<<count<<endl;


                _v_q->data = v_app;
                
                vv =  _v_q->data + q0->data;

                if(v_app(0) < (29.9*3.14/180)){
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
			cmd[i].data = v_app(i);
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
        cout<<"ERG CONTROL PRESS 1 TO START"<<endl;
        cin>>x;
	ros::init(argc, argv, "iiwa_kdl_erg");
	KUKA_INVKIN ik;
	ik.run();

	return 0;
}
