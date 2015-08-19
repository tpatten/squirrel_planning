#include <ros/ros.h>
#include <squirrel_prediction_msgs/RecommendRelations.h>

#include <Python.h>

using namespace std;

int callRecommender(std::string pythonPath, std::string pythonFile, std::string pythonFun, std::string trainedPath, std::string passedFilePath);

 bool uibk_recommend(squirrel_prediction_msgs::RecommendRelations::Request  &req,
          squirrel_prediction_msgs::RecommendRelations::Response &res){

   int classifierRes = callRecommender("/home/c7031098/squirrel_ws/src/squirrel_planning/squirrel_recommender/scripts/mmr_mmmvr",
                                       "kingsc_main", "runRecommender",
                                         "/home/c7031098/squirrel_ws/src/squirrel_planning/squirrel_recommender/scripts/mmr_mmmvr",
                                          "/home/c7031098/squirrel_ws/src/squirrel_planning/squirrel_recommender/scripts/mmr_mmmvr");

   //res.sum = req.a + req.b;
   //ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
   //ROS_INFO("sending back response: [%ld]", (long int)res.sum);
   return true;
 }


int main( int argc, char *argv[] ){
  ros::init(argc, argv, "recommender");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("recommender", uibk_recommend);
  ROS_INFO("(squirrel recommender) Ready for predicting missing values.");
  ros::spin();

  return 0;
}

int callRecommender(std::string pythonPath, std::string pythonFile, std::string pythonFun, std::string trainedPath, std::string passedFilePath) {

    int retVal = -1;
    string mName = pythonFile;
    string fName = pythonFun;
    string argumentVal = trainedPath;

    PyObject *pName, *pModule, *pFunc;
    PyObject *pArgs, *pValue;

    Py_Initialize();
    PyRun_SimpleString("import sys");
    PyRun_SimpleString(string(string("sys.path.append('") + pythonPath + string("')")).c_str());
    PyRun_SimpleString("import kingsc_main");

    pName = PyUnicode_FromString(mName.c_str());
    pModule = PyImport_Import(pName);
    Py_DECREF(pName);

    if (pModule != NULL) {

        pFunc = PyObject_GetAttrString(pModule, fName.c_str());

        if (pFunc && PyCallable_Check(pFunc)) {

            pArgs = PyTuple_New(2);
            
              
            pValue = PyUnicode_FromString(argumentVal.c_str());
            if (!pValue) {
                Py_DECREF(pArgs);
                Py_DECREF(pModule);
                fprintf(stderr, "Cannot convert argument\n");
                return 1;
            }
            PyTuple_SetItem(pArgs, 0, pValue);

            pValue = PyUnicode_FromString(passedFilePath.c_str());
            if (!pValue) {
                Py_DECREF(pArgs);
                Py_DECREF(pModule);
                fprintf(stderr, "Cannot convert argument\n");
                return 1;
            }
            PyTuple_SetItem(pArgs, 1, pValue);
            
            pValue = PyObject_CallObject(pFunc, pArgs);
            Py_DECREF(pArgs);
            if (pValue != NULL) {
                retVal = PyLong_AsLong(pValue);
                Py_DECREF(pValue);
            }
            else {
                Py_DECREF(pFunc);
                Py_DECREF(pModule);
                PyErr_Print();
                fprintf(stderr,"Call failed\n");
                retVal = -1;
            }

        }
        else {
            if (PyErr_Occurred())
                PyErr_Print();
            cerr << "Cannot find function " << fName << endl;
        }
        Py_XDECREF(pFunc);
        Py_DECREF(pModule);
    }
    else {
        PyErr_Print();
        cerr << "Failed to load " << mName << endl;
        retVal = -1;
    }
    Py_Finalize();
    return retVal;

}


