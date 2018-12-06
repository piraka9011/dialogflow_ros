#include <ros/ros.h>
#include <dialogflow_ros/DialogflowEvent.h>
#include <dialogflow_ros/DialogflowParameter.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_intent_event");
	ros::NodeHandle n;
	ros::Rate poll_rate(100);
	ros::Publisher pub = n.advertise<dialogflow_ros::DialogflowEvent>("/dialogflow_client/requests/df_event", 10);
	while(pub.getNumSubscribers() == 0) poll_rate.sleep();
	ros::Duration(1).sleep();

	dialogflow_ros::DialogflowEvent event;
	event.event_name = "objects_found";
	dialogflow_ros::DialogflowParameter parameter;
	parameter.param_name = "objects";
	std::string milk = "milk";
	std::string snack = "snack";
	std::vector<std::string> object_list = {milk, snack};
	// parameter.value.resize(2);
	parameter.value = object_list;
	// parameter.value.push_back(milk);
	// parameter.value = object_list;
	event.parameters.push_back(parameter);

	// std::cout << "Event: " << event << std::endl;
	std::cout << "Parameter: " << parameter << std::endl;
	std::cout << "Values" << std::endl;
	for (auto val : parameter.value)
		std::cout << val << std::endl;
	
	
	pub.publish(event);
	ros::Duration(1).sleep();
	return 0;
}