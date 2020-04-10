#ifndef ROSNEURO_ACQUISITION_HPP
#define ROSNEURO_ACQUISITION_HPP

#include <ros/ros.h>
#include "rosneuro_msgs/NeuroEvent.h"
#include "rosneuro_cybathlon/udp_client_server.hpp"

#define NUM_COMMANDS 			2

int BCI_COMMANDS [2] = {773, 771};
int COMMAND = 899;

namespace rosneuro {

class Control {
	public:
		Control(void);
		virtual ~Control(void);

		bool configure(void);
		bool Run(void);
		unsigned int GetPlayerId(char* playerStr);

	private:
		void on_received(const rosneuro_msgs::NeuroEvent::ConstPtr& msg);
		unsigned int Event2Command(int idevent, unsigned int player);
		int FindCommand(int event);

	private:
		ros::NodeHandle							nh_;
		std::string								topic_;
		ros::Subscriber							sub_;
		ros::WallTime 							start_, end_;
		
		udp_client_server::udp_client 			CybGame_;
		std::string								server_ip_;
		int 									port_;
		
		unsigned int 		player_;
		bool 				cmdflag_;
		bool				eogdetected_;
		float				reversetime_;
		int 				bcievt_p_;
		int 				bcievt_;
		int 				cmdevt_;
		float				bcitime_;
		int 				idevt_;
		int 				chkevt_;

};



}

#endif
