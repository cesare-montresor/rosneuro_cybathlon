#ifndef ROSNEURO_CYBATHLON_CONTROL_CPP
#define ROSNEURO_CYBATHLON_CONTROL_CPP

#include "rosneuro_cybathlon/Control.hpp"

namespace rosneuro {

Control::Control(void) : nh_("~") {
	this->topic_	 	= "/events/bus";
	this->cmdflag_	 	= false;
	this->eogdetected_ 	= false;
	this->bcievt_p_		= 0;
}

Control::~Control(void) {}

bool Control::configure(void) {

	ros::param::param<std::string>("~server_ip", this->server_ip_, "127.0.0.1");
	ros::param::param("~port", this->port_, 5555);
	ros::param::param("~player", (int&) this->player_, 1);
	ros::param::param("~reverse_time", this->reversetime_, 2000.0f); // in milliseconds

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
	ROS_INFO("Cybathlon Player Id: %d", this->player_);

	while(this->CybGame_.connect(this->server_ip_, this->port_) == -1) {
		ROS_INFO("Waiting for connection to the game (%s)...", this->server_ip_.c_str());
		ros::Duration(1.0).sleep();
	}
	ROS_INFO("Connection to the game (%s) established", this->server_ip_.c_str());

	this->start_ = ros::WallTime::now();
	ROS_INFO("Control started");
	while(this->nh_.ok() && quit == false) {
	
 	  if (this->cmdflag_ == true) {
	
	     if(this->bcitime_ <= this->reversetime_ && this->bcievt_p_ != this->bcievt_) {
		    this->cmdevt_ = 770;
		    ROS_INFO("Reverse command");
		    } 
	     else {
		    this->cmdevt_ = this->bcievt_;
	      }
  
	     if (this->eogdetected_ == false) {
			unsigned int cmdcyb = this->Event2Command(this->cmdevt_, this->player_);
			for (auto i=0; i<3; i++) {
				this->CybGame_.send((const void*)&cmdcyb,sizeof(unsigned int));
				ros::Duration(0.025).sleep();
			}
			ROS_INFO("Command sent: %d", cmdcyb);
		}
	     this->cmdflag_ = false;

	   } 

	  ros::spinOnce();
	  r.sleep();

	}
	ROS_INFO("Cybathlon control closed"); 

	return true;
}

void Control::on_received(const rosneuro_msgs::NeuroEvent::ConstPtr& msg) {

	
	this->idevt_ = msg->event;
	
	 this->chkevt_ = this->FindCommand(this->idevt_);

	 if(this->chkevt_!=-1)
	 {
	    this->bcievt_p_ = this->bcievt_;	
	    this->bcievt_ = this->chkevt_;
	    this->end_ = ros::WallTime::now();
	    this->bcitime_ = (this->end_ - this->start_).toNSec() * 1e-6;
	    ROS_INFO("BCI command received: %d (after %f ms)", this->bcievt_, this->bcitime_);
	    this->start_ = ros::WallTime::now();
	    this->cmdflag_  = true;

	 }

	
	// Check EOG

	 if(this->idevt_==1024)
	 {
		this->eogdetected_ = true;
		ROS_INFO("eog detected | command disabled (event: %d)", this->idevt_);
		
	 }
	 else if(this->idevt_ == 33792) {
	        this->eogdetected_ = false;
	        ROS_INFO("eog timeout elapsed | command enabled (event: %d)", this->idevt_);
		
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



int Control::FindCommand(int event) {	
	
	for(int i=0; i< sizeof(BCI_COMMANDS)/sizeof(int); i++) {
		if(BCI_COMMANDS[i] == event - COMMAND)
		   return BCI_COMMANDS[i];
	}

	return -1;
	
}


}



#endif
