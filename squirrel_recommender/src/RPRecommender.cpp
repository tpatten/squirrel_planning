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

	/*----------*/
	/* read CSV */
	/*----------*/

	/**
	 * Read from a CSV file and write new facts to the KMS
	 */
	bool RPRecommender::readDatabase(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {

		domain_types.clear();
		domain_attributes.clear();
		current_instances.clear();
		all_instances.clear();
		current_attributes.clear();

		ros::NodeHandle nh("~");

		// fetch knowledge
		ROS_INFO("KCL: (RPRecommender) Reading database");

		std::stringstream ss;
		ss << data_path << "predicted_missing_known_full.csv";
		std::ifstream pFile;
		pFile.open(ss.str().c_str());

		std::string line;
		std::vector<std::string> str_list;
		std::string preds[] = {"can_fit_inside","can_pickup","can_push","can_stack_on"};
		std::string vars[][2] = {{"o","b"},{"v","o"},{"v","o"},{"o1","o2"}};

		if (pFile.is_open()) {

			// header line # object1, object2, features, confidences
			if(!pFile.eof()) std::getline(pFile,line);	

			while(!pFile.eof()) {

				// parse line
				str_list.clear();
				for(int i=0;i<6;i++) {
					std::getline(pFile,line,',');
					str_list.push_back(line);
				}
				std::getline(pFile,line);

				// 'object1','object2','can_fit_inside','can_pickup','can_push','can_stack_on','confidences'
				for(int i=0;i<6;i++) {

					if("1" == str_list[i]) {
						// Add proposition TODO check confidence
						rosplan_knowledge_msgs::KnowledgeUpdateService propSrv;
						propSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
						propSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
						propSrv.request.knowledge.attribute_name = preds[(i-2)];
						diagnostic_msgs::KeyValue oPair;
						oPair.key = vars[(i-2)][0];
						oPair.value = str_list[0].substr(1,str_list[0].length()-2);
						propSrv.request.knowledge.values.push_back(oPair);
						diagnostic_msgs::KeyValue oPair2;
						oPair2.key = vars[(i-2)][1];
						oPair2.value = str_list[1].substr(1,str_list[1].length()-2);
						propSrv.request.knowledge.values.push_back(oPair2);
						if (!update_knowledge_client.call(propSrv)) {
							ROS_ERROR("KCL: (ObjectPerception) error adding object_at predicate");
							return false;
						}
					}
				}
			}
		}
		pFile.close();

		ROS_INFO("KCL: (RPRecommender) done");
		return true;
	}

	/*-----------*/
	/* build CSV */
	/*-----------*/
	
	/**
	 * Read from the KMS and generate a csv file for the learning
	 */
	bool RPRecommender::initialiseDatabase(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {

		domain_types.clear();
		domain_attributes.clear();
		current_instances.clear();
		all_instances.clear();
		current_attributes.clear();

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
		ss << data_path << "known.csv";
		std::ofstream pFile;
		pFile.open(ss.str().c_str());

		pFile << "'object1','object2'";
		for(int i=0; i<domain_attributes.size(); i++) {
			if(domain_attributes[i].name=="object_at"
					|| domain_attributes[i].name=="on"
					|| domain_attributes[i].name=="holding"
					|| domain_attributes[i].name=="inside"
					|| domain_attributes[i].name=="box_at"
					|| domain_attributes[i].name=="connected"
					|| domain_attributes[i].name=="tidy_location"
					|| domain_attributes[i].name=="robot_at"
					|| domain_attributes[i].name=="push_location")
				continue;
			pFile << ",'" << domain_attributes[i].name << "'";
		}
		pFile << std::endl;

		for(int i=0; i<all_instances.size(); i++) {

			if(all_instances[i].substr(0,2)=="wp") continue;

		for(int j=0; j<all_instances.size(); j++) {

			if(all_instances[j].substr(0,2)=="wp") continue;

			pFile << "'" << all_instances[i] << "','" << all_instances[j] << "'";
			for(int k=0; k<domain_attributes.size(); k++) {

				// 'holding','inside','on' ??
				if(domain_attributes[k].name=="object_at"
						|| domain_attributes[k].name=="on"
						|| domain_attributes[k].name=="holding"
						|| domain_attributes[k].name=="inside"
						|| domain_attributes[k].name=="box_at"
						|| domain_attributes[k].name=="connected"
						|| domain_attributes[k].name=="tidy_location"
						|| domain_attributes[k].name=="robot_at"
						|| domain_attributes[k].name=="push_location")
					continue;

				if(attributeFalse(current_instances[all_instances[i]], current_instances[all_instances[j]], domain_attributes[k]))
					pFile << ",0";
				else if(attributeTrue(all_instances[i], all_instances[j], domain_attributes[k].name))
					pFile << ",1";
				else
					pFile << ",";
			}
			pFile << std::endl;
		}};

		pFile.close();

		// send to recommender
		ROS_INFO("KCL: (RPRecommender) Calling recommender");

		squirrel_prediction_msgs::RecommendRelations rrSrv;
		rrSrv.request.inputFile = ss.str();
		std::stringstream outputFileName;
		outputFileName << data_path << "predicted_missing_known_full.csv";
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
		ros::ServiceServer readdbService = nh.advertiseService("/kcl_rosplan/recommender/read_db", &KCL_rosplan::RPRecommender::readDatabase, &rrm);

		ROS_INFO("KCL: (RPRecommender) Ready to receive.");
		ros::spin();
		return 0;
	}
