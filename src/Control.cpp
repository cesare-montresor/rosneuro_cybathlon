#ifndef ROSNEURO_CYBATHLON_CONTROL_CPP
#define ROSNEURO_CYBATHLON_CONTROL_CPP

#include "rosneuro_cybathlon/Control.hpp"

namespace rosneuro {

Control::Control(void) : nh_("~") {
	this->topic_	 	= "/classifier/output";
	this->cmdflag	 	= false;
	this->eogdetected 	= false;
	this->bcievt_p		= 0;
}

Control::~Control(void) {}

bool Control::configure(void) {

	ros::param::param("~server_ip", this->server_ip_, "127.0.0.1");
	ros::param::param("~port", this->port_, 5555);
	ros::param::param("~player", this->player_, 1);
	ros::param::param("~reverse_time", this->reversetime, 2);

	this->sub_ = this->nh_.subscribe(this->topic_, 1000, &Control::on_received, this);


	return true;
}

bool Control::Run(void) {

	bool quit = false;
	ros::Rate r(100);

	// Configure control
	if(this->configure() == false) {
		ROS_ERROR("Cannot configure the Cybathlon control");
		return false;
	}
	ROS_INFO("Control correctly configured");

	//**** Configuring Game connection ****/	
	ROS_INFO("Cybathlon Player Id: " << this->player_);

	while(this->CybGame_.connect(this->server_ip_, this->port_) == -1) {
		ROS_INFO("Waiting for connection to the game ("<<this->server_ip_<<")...");
		ros::Duration(1.0).sleep();
	}
	ROS_INFO("Connection to the game ("<<this->server_ip_<<") established");

	this->start_ = ros::WallTime::now();
	ROS_INFO("Control started");
	while(this->nh_.ok() && quit == false) {
	
		ros::spinOnce();
		r.sleep();

	}
	ROS_INFO("Cybathlon control closed");

	return true;
}

void Control::on_received(const rosneuro_msgs::NeuroOutput& msg) {

	unsigned int idevent = msg.hardpredict.data;
	for (int i = 0; i < NUM_COMMANDS; ++i) {
		if (msg.hardpredict.data == BCI_COMMANDS[i]) {
			this->cmdflag_ = true;
		}
	}

	if (this->cmdflag_ == true) {
		unsigned int bcievt = idevent;
		this->end_ = ros::WallTime::now();
		double bcitime = (this->end_ - this->start_).toNSec() * 1e-6;
		ROS_INFO("BCI command received: "<<bcievt<<" (after "<<bcitime<<" ms)");
		this->start_ = ros::WallTime::now();
	} else {
		// EOG detection
		if(idevent == 1024) {
			this->eogdetected_ = true;
			ROS_INFO("eog detected | command disabled (event: "<< idevent<<")");
		} else if(idevent == 33792) {
			this->eogdetected_ = false;
			ROS_INFO("eog timeout elapsed | command enabled (event: "<< idevent<<")");
		}
	}

	if (this->cmdflag_ == true) {
			if(bcitime <= this->reversetime_ && this->bcievt_p_ != bcievt) {
				unsigned int cmdevt = 770;
				ROS_INFO("Reverse command");
			} else {
				unsigned int cmdevt = bcievt;
			}
		}

	if(this->cmdflag_ == true)  {
		if (this->eogdetected_ == false) {
			unsigned int cmdcyb = this->Event2Command(cmdevt, this->player_);
			for (auto i=0; i<3; i++) {
				this->CybGame_.send((const void*)&cmdcyb,sizeof(unsigned int));
				ros::Duration(0.025).sleep();
			}
			ROS_INFO("Command sent: " << cmdcyb);
		}
		this->cmdflag_ = false;
	}

}

unsigned int Control::Event2Command(int idevent, unsigned int player) {

	unsigned int command = -1;

	switch(idevent) {
		case 773:
			command = 1 + 10 * player;
			break;
		case 770:
			command = 2 + 10 * player;
			break;
		case 771:
			command = 3 + 10 * player;
			break;
		default:
			break;
	}

	return command;
}

unsigned int Control::GetPlayerId(char* playerStr) {
	unsigned int playerId;
	playerId = std::stoi(playerStr);
	
	return playerId;
}


}



#endif