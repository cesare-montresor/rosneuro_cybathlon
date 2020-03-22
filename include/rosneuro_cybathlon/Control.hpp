#ifndef ROSNEURO_ACQUISITION_HPP
#define ROSNEURO_ACQUISITION_HPP

#include <ros/ros.h>
#include "rosneuro_msgs/NeuroOutput.h"
#include "rosneuro_cybathlon/udp_client_server.hpp"

#define NUM_COMMANDS 			2
#define BCI_COMMANDS 	{771, 773}

namespace rosneuro {

class Control {
	public:
		Control(void);
		virtual ~Control(void);

		bool configure(void);
		bool Run(void);
		unsigned int GetPlayerId(char* playerStr);

	private:
		void on_received(const rosneuro_msgs::NeuroOutput& msg);
		unsigned int Event2Command(int idevent, unsigned int player);

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
		unsigned int 		bcievt_p_;

};



}

#endif