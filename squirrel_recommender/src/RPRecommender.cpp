#include "squirrel_recommender/RPRecommender.h"

/* implementation of rosplan_interface_recommender::RPRecommender */
namespace KCL_rosplan {

	/* constructor */
	RPRecommender::RPRecommender(ros::NodeHandle &nh) {

		// config
		std::string dataPath("common/");
		nh.param("data_path", data_path, dataPath);

		// knowledge interface
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
		domain_types_client = nh.serviceClient<rosplan_knowledge_msgs::GetDomainTypeService>("/kcl_rosplan/get_domain_types");
		domain_attributes_client = nh.serviceClient<rosplan_knowledge_msgs::GetDomainAttributeService>("/kcl_rosplan/get_domain_predicates");
		current_instances_client = nh.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_current_instances");
		current_goals_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_goals");
		current_attributes_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_knowledge");
		recommender_client = nh.serviceClient<squirrel_prediction_msgs::RecommendRelations>("/recommender");
	}

	/*-----------*/
	/* build PRM */
	/*-----------*/
	
	/**
	 * Generates waypoints and stores them in the knowledge base and scene database
	 */
	bool RPRecommender::initialiseDatabase(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {

		ros::NodeHandle nh("~");

		// fetch knowledge
		ROS_INFO("KCL: (RPRecommender) Fetching knowledge");

		rosplan_knowledge_msgs::GetDomainTypeService gdt;
		domain_types_client.call(gdt);
		for(int i=0; i<gdt.response.types.size(); i++) {
			domain_types.push_back(gdt.response.types[i]);
		}

		rosplan_knowledge_msgs::GetDomainAttributeService gda;
		domain_attributes_client.call(gda);
		for(int i=0; i<gda.response.items.size(); i++) {
			if(gda.response.items[i].typed_parameters.size()==2)
				domain_attributes.push_back(gda.response.items[i]);
		}

		for(int i=0; i<gdt.response.types.size(); i++) {
			rosplan_knowledge_msgs::GetInstanceService gci;
			gci.request.type_name = gdt.response.types[i];
			current_instances_client.call(gci);
			for(int j=0; j<gci.response.instances.size(); j++) {
				current_instances[gci.response.instances[j]] = gdt.response.types[i];
				all_instances.push_back(gci.response.instances[j]);
			}
		}

		for(int i=0; i<domain_attributes.size(); i++) {
			rosplan_knowledge_msgs::GetAttributeService gca;
			gca.request.predicate_name = domain_attributes[i].name;
			current_attributes_client.call(gca);
			for(int j=0; j<gca.response.attributes.size(); j++) {
				current_attributes[domain_attributes[i].name].push_back(gca.response.attributes[j]);
			}
		}

		// create csv file
		ROS_INFO("KCL: (RPRecommender) Creating file");

		std::stringstream ss;
		ss << data_path << "input.csv" << std::endl;
		std::ofstream pFile;
		pFile.open(ss.str().c_str());

		pFile << "object1, object2";
		for(int i=0; i<domain_attributes.size(); i++)
			pFile << ", " << domain_attributes[i].name;
		pFile << std::endl;

		for(int i=0; i<all_instances.size(); i++) {
		for(int j=0; j<all_instances.size(); j++) {
			pFile << all_instances[i] << ", " << all_instances[j];
			for(int k=0; k<domain_attributes.size(); k++) {
				if(attributeFalse(current_instances[all_instances[i]], current_instances[all_instances[j]], domain_attributes[k]))
					pFile << ", 0";
				else if(attributeTrue(all_instances[i], all_instances[j], domain_attributes[k].name))
					pFile << ", 1";
				else
					pFile << ", ";
			}
			pFile << std::endl;
		}};

		pFile.close();

		// send to recommender
		ROS_INFO("KCL: (RPRecommender) Calling recommender");

		squirrel_prediction_msgs::RecommendRelations rrSrv;
		rrSrv.request.inputFile = ss.str();
		std::stringstream outputFileName;
		outputFileName << data_path << "output.csv" << std::endl;
		rrSrv.request.outputFile = ss.str();
		rrSrv.request.initilization = true;
		
		recommender_client.call(rrSrv);

		ROS_INFO("KCL: (RPRecommender) Done");
		return true;
	}

	bool RPRecommender::attributeTrue(std::string instanceA, std::string instanceB, std::string attributeName) {
		for(int i=0;i<current_attributes[attributeName].size(); i++) {
			if(current_attributes[attributeName][i].values[0].value==instanceA
					&& current_attributes[attributeName][i].values[1].value==instanceB)
				return true;
		}
		return false;
	}

	bool RPRecommender::attributeFalse(std::string typeA, std::string typeB, rosplan_knowledge_msgs::DomainFormula attribute) {
		return (attribute.typed_parameters[0].value!=typeA
				|| attribute.typed_parameters[1].value!=typeB);
			
	}

} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		// setup ros
		ros::init(argc, argv, "rosplan_interface_recommender");
		ros::NodeHandle nh("~");

		// init
		KCL_rosplan::RPRecommender rrm(nh);
		ros::ServiceServer initdbService = nh.advertiseService("/kcl_rosplan/recommender/init_db", &KCL_rosplan::RPRecommender::initialiseDatabase, &rrm);

		ROS_INFO("KCL: (RPRecommender) Ready to receive.");
		ros::spin();
		return 0;
	}
