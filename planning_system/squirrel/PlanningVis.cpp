#include "ros/ros.h"

#include "planning_dispatch_msgs/ActionDispatch.h"
#include "planning_dispatch_msgs/ActionFeedback.h"

#include <GL/freeglut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glx.h>

#include <iostream>
#include <string>
#include <stdio.h>
#include <string.h>

#define SPACE 32

namespace KCL_squirrelVIS {

	unsigned int WINDOW_WIDTH = 500;
	unsigned int WINDOW_HEIGHT = 500;
	unsigned int BOX_SIZE  = 50;

	std::string actions[] = { "explore", "classify", "push" };
	std::map<int,std::string> currentActions;
	std::map<int,bool> cancelledActions;

	int window;
	GLuint base;

	ros::Publisher pausePublisher;

	/* The function called whenever a key is pressed. */
	void keyPressed(unsigned char key, int x, int y) {
		if (key == SPACE) {
			std_msgs::Empty msg;
			pausePublisher.publish(msg);
		}
		// avoid thrashing this procedure
		usleep(100);
	}

	/*-----------*/
	/* shut down */
	/*-----------*/

	void shutDownVis() {
		glutDestroyWindow(window);
		glDeleteLists(base, 96);
	}

	/*---------------*/
	/* init graphics */
	/*---------------*/

	/* build font */
	void buildFont() {

		Display *dpy;
		XFontStruct *fontInfo;
		base = glGenLists(96);

		// default to DISPLAY env.
		dpy = XOpenDisplay(NULL);

		fontInfo = XLoadQueryFont(dpy, "-adobe-helvetica-medium-r-normal--18-*-*-*-p-*-iso8859-1");
		if (fontInfo == NULL) fontInfo = XLoadQueryFont(dpy, "fixed");
		if (fontInfo == NULL) std::cout << "no X font available?" << std::endl;

		glXUseXFont(fontInfo->fid, 32, 96, base);
		XFreeFont(dpy, fontInfo);
		XCloseDisplay(dpy);
	}

	void updateCamera() {
		glViewport(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);
		glMatrixMode(GL_MODELVIEW);

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(0, WINDOW_WIDTH, WINDOW_HEIGHT, 0, 1, -1);
		glMatrixMode(GL_MODELVIEW);
	}

	/* OpenGL initialization */
	void initVis() {

		char appname[] = "SQUIRREL TIDY-NATOR 500: Prince of Push";
		char *fake_argv[] = { appname, NULL };
		int fake_argc = 1;
		glutInit(&fake_argc, fake_argv);  
		glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA);  

		glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
		glutInitWindowPosition(0, 0);  
		window = glutCreateWindow("SQUIRREL TIDY-NATOR 500: Prince of Push");  

		glutKeyboardFunc(&keyPressed);

 		buildFont();

		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

		glEnable(GL_TEXTURE_2D);
		glShadeModel(GL_SMOOTH);
		glDisable(GL_DEPTH_TEST);
		glDisable(GL_LIGHTING);

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glEnable(GL_ALPHA_TEST);
		glAlphaFunc(GL_GREATER, 0.1f);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

		updateCamera();
	}

	/*--------------*/
	/* draw methods */
	/*--------------*/

	void drawText(const std::string &textstring, int size, float x, float baseline, float r, float g, float b) {

		const char* text = textstring.c_str();

		glPushMatrix();
		glDisable(GL_TEXTURE_2D);
		glLoadIdentity();

		glColor4f(r,g,b,1.0f);
		glTranslatef(x, baseline+size, 0.0);
		glRasterPos2f(0.0, 0.0);

		int length = (int) std::strlen(text);
		for (int i = 0; i < length; i++) {
			if(size==12) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, text[i]);
			if(size==18) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, text[i]);
		}

		glEnable(GL_TEXTURE_2D);
		glPopMatrix();
	}

	void drawRect(float x, float y, float width, float height, float r, float g, float b) {

		glPushMatrix();
		glBindTexture(GL_TEXTURE_2D, 0);

		glColor4f(r,g,b,1.0f);
		glTranslatef(x, y, 0);		

		glBegin(GL_LINE_LOOP);
		{
			glVertex2f(0, 0);
			glVertex2f(0, height);
			glVertex2f(width,height);
			glVertex2f(width,0);
		}
		glEnd();
		glPopMatrix();
	}

	void fillRect(float x, float y, float width, float height, float r, float g, float b) {

		glPushMatrix();
		glBindTexture(GL_TEXTURE_2D, 0);

		glColor4f(r,g,b,1.0f);
		glTranslatef(x, y, 0);		

		glBegin(GL_POLYGON);
		{
			glVertex2f(0, 0);
			glVertex2f(0, height);
			glVertex2f(width,height);
			glVertex2f(width,0);
		}
		glEnd();
		glPopMatrix();
	}

	void drawAction(std::string &name, bool active, bool cancelled, int x, int y) {

		float border = BOX_SIZE/20.0f;

		fillRect(x,y,BOX_SIZE*4,BOX_SIZE,0.0f,0.0f,0.0f);
		if(!active)
			fillRect(x+border, y+border, BOX_SIZE*4-2*border, BOX_SIZE-2*border, 1.0f,1.0f,1.0f);
		else {
			if(cancelled) fillRect(x+border, y+border, BOX_SIZE*4-2*border, BOX_SIZE-2*border, 1.0f,0.3f,0.3f);
			else fillRect(x+border, y+border, BOX_SIZE*4-2*border, BOX_SIZE-2*border, 0.3f,1.0f,0.3f);
		}

		drawText(name, 18, x+border*2, y+border*2, 0.0f, 0.0f, 0.0f);
	}

	/* The main drawing function */
	void draw() {

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glLoadIdentity();

		WINDOW_WIDTH = glutGet(GLUT_WINDOW_WIDTH);
		WINDOW_HEIGHT = glutGet(GLUT_WINDOW_HEIGHT);
		BOX_SIZE = WINDOW_HEIGHT/6;
		updateCamera();

		fillRect(0,0, WINDOW_WIDTH, WINDOW_HEIGHT, 1.0f,1.0f,1.0f);
		
		for(size_t i=0;i<3;i++) {

			bool active = false;
			bool cancelled = false;
			for(std::map<int,std::string>::iterator ait=currentActions.begin(); ait!=currentActions.end(); ait++) {
				if(ait->second.compare(actions[i])==0) {
					active = true;
					cancelled = cancelledActions[ait->first];
				}
			}
			drawAction(actions[i], active, cancelled, (WINDOW_WIDTH-4*BOX_SIZE)/2, BOX_SIZE*(1+i*3)/2);
		}

		glutSwapBuffers();
		glutMainLoopEvent();
	}

	void runVis() {
		ros::Rate loop_rate(20);
		while(ros::ok() && glutGetWindow()!=0) {
			draw();
			ros::spinOnce();
			loop_rate.sleep();
		}
	}	

	/*-----------------*/
	/* ROS subscribers */
	/*-----------------*/

	void dispatchVisCallback(const planning_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		std::cout << msg->name << std::endl;
		if(msg->name.compare("cancel_action")==0) {
			cancelledActions[msg->action_id] = true;
			std::cout << "cancelled " << msg->action_id << std::endl;
		} else {
			currentActions[msg->action_id] = msg->name;
			cancelledActions[msg->action_id] = false;
		}
	}

	void feedbackVisCallback(const planning_dispatch_msgs::ActionFeedback::ConstPtr& msg) {
		if(msg->status.compare("action achieved")==0) {
			currentActions[msg->action_id] = "";
			cancelledActions[msg->action_id] = false;
		}
	}
}

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc,argv,"planning_ros_visualisation");

		ros::NodeHandle nh("~");
		ros::Subscriber dispatchVisSub = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, KCL_squirrelVIS::dispatchVisCallback);
		ros::Subscriber feedbackVisSub = nh.subscribe("/kcl_rosplan/action_feedback", 1000, KCL_squirrelVIS::feedbackVisCallback);

		KCL_squirrelVIS::pausePublisher = nh.advertise<std_msgs::Empty>("/kcl_rosplan/pause_dispatch", 1000, true);

		KCL_squirrelVIS::initVis();
		KCL_squirrelVIS::runVis();

		return 0;
	}
