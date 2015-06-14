#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <fl/Headers.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

using namespace std;

float yfinall = 0;
float xfinall = 0;
float usrc = 0;
bool calibration = 0;

int main(int argc, char** argv){
    using namespace fl;
    Engine* engine = new Engine("Controle Lateral");
    Engine* engine2 = new Engine("Controle Linear");

    InputVariable* xtotal = new InputVariable;
    OutputVariable* Vangular = new OutputVariable;
    InputVariable* ytotal = new InputVariable;
    OutputVariable* Vlinear = new OutputVariable;

  ros::init(argc, argv, "gesture_teleop");

  ros::NodeHandle node;

  ros::Publisher cmd_vel_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  
  tf::TransformListener listener;

  geometry_msgs::Twist msgv;

  int mycount = 0;
  ros::Rate rate(10.0);

  while (node.ok()){
    tf::StampedTransform transform;
    tf::StampedTransform transform2;
    try{
      listener.lookupTransform("/torso_1", "/left_hand_1",
                               ros::Time(0), transform);
      listener.lookupTransform("/torso_1", "/right_hand_1",
                               ros::Time(0), transform2);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
    }

     if (calibration == 0){	
     	if (mycount == 0){
    	 FL_LOG("Please stand in the cross position in front of Kinect for Fuzzy calibration! (10 seconds)");
	}	
     	if (mycount < 100){
	if (transform.getOrigin().x() >= -(transform2.getOrigin().x())){
		if (transform.getOrigin().x() > (usrc+0.2)){
			usrc = transform.getOrigin().x()-0.2;
		}
	}
	if (transform.getOrigin().x() < -(transform2.getOrigin().x())){
		if (-(transform2.getOrigin().x()) > (usrc+0.2)){
			usrc = -(transform2.getOrigin().x())-0.2;
		}
	}
//     ROS_INFO("%.6f", usrc);
	}
     	if (mycount == 100){

// CALIBRACAO

    xtotal->setName("xtotal");
    xtotal->setRange(-usrc, usrc);
    xtotal->addTerm(new Triangle("GN", -usrc, -((usrc+0.1)/2)));
    xtotal->addTerm(new Triangle("MN", -(((usrc+0.1)/2)+((usrc-0.1)/3)), -(((usrc+0.1)/2)-((usrc-0.1)/3))));
    xtotal->addTerm(new Triangle("PN", -((usrc+0.1)/2), -0.100));
    xtotal->addTerm(new Triangle("P", 0.100, ((usrc+0.1)/2)));
    xtotal->addTerm(new Triangle("M", (((usrc+0.1)/2)-((usrc-0.1)/3)), (((usrc+0.1)/2)+((usrc-0.1)/3))));
    xtotal->addTerm(new Triangle("G", ((usrc+0.1)/2), usrc));
    engine->addInputVariable(xtotal);

    Vangular->setName("Vangular");
    Vangular->setRange(-0.900, 0.900);
    Vangular->setDefaultValue(fl::nan);
    Vangular->addTerm(new Triangle("ALTAN", -0.900, -0.300));
    Vangular->addTerm(new Triangle("MEDIAN", -0.600, -0.000));
    Vangular->addTerm(new Triangle("BAIXA", -0.300, 0.300));
    Vangular->addTerm(new Triangle("MEDIA", 0.000, 0.600));
    Vangular->addTerm(new Triangle("ALTA", 0.300, 0.900));
    engine->addOutputVariable(Vangular);

    ytotal->setName("ytotal");
    ytotal->setRange(-0.450, 0.450);
    ytotal->addTerm(new Triangle("GN", -0.450, -0.280));
    ytotal->addTerm(new Triangle("MN", -0.370, -0.190));
    ytotal->addTerm(new Triangle("PN", -0.280, -0.100));
    ytotal->addTerm(new Triangle("P", 0.100, 0.280));
    ytotal->addTerm(new Triangle("M", 0.190, 0.370));
    ytotal->addTerm(new Triangle("G", 0.280, 0.450));
    engine2->addInputVariable(ytotal);

    Vlinear->setName("Vlinear");
    Vlinear->setRange(-1.500, 1.500);
    Vlinear->setDefaultValue(fl::nan);
    Vlinear->addTerm(new Triangle("ALTAN", -1.500, -0.500));
    Vlinear->addTerm(new Triangle("MEDIAN", -1.000, -0.000));
    Vlinear->addTerm(new Triangle("BAIXA", -0.500, 0.500));
    Vlinear->addTerm(new Triangle("MEDIA", 0.000, 1.000));
    Vlinear->addTerm(new Triangle("ALTA", 0.500, 1.500));
    engine2->addOutputVariable(Vlinear);

    RuleBlock* ruleblock = new RuleBlock;
    ruleblock->addRule(Rule::parse("if xtotal is GN then Vangular is ALTAN", engine));
    ruleblock->addRule(Rule::parse("if xtotal is MN then Vangular is MEDIAN", engine));
    ruleblock->addRule(Rule::parse("if xtotal is PN then Vangular is BAIXA", engine));
    ruleblock->addRule(Rule::parse("if xtotal is P then Vangular is BAIXA", engine));
    ruleblock->addRule(Rule::parse("if xtotal is M then Vangular is MEDIA", engine));
    ruleblock->addRule(Rule::parse("if xtotal is G then Vangular is ALTA", engine));
    engine->addRuleBlock(ruleblock);

    RuleBlock* ruleblock2 = new RuleBlock;
    ruleblock2->addRule(Rule::parse("if ytotal is GN then Vlinear is ALTAN", engine2));
    ruleblock2->addRule(Rule::parse("if ytotal is MN then Vlinear is MEDIAN", engine2));
    ruleblock2->addRule(Rule::parse("if ytotal is PN then Vlinear is BAIXA", engine2));
    ruleblock2->addRule(Rule::parse("if ytotal is P then Vlinear is BAIXA", engine2));
    ruleblock2->addRule(Rule::parse("if ytotal is M then Vlinear is MEDIA", engine2));
    ruleblock2->addRule(Rule::parse("if ytotal is G then Vlinear is ALTA", engine2));
    engine2->addRuleBlock(ruleblock2);

    engine->configure("", "", "Minimum", "Maximum", "Centroid");
    engine2->configure("", "", "Minimum", "Maximum", "Centroid");

    std::string status;
    if (not engine->isReady(&status))
        throw Exception("Engine not ready. "
            "The following errors were encountered:\n" + status, FL_AT);
    if (not engine2->isReady(&status))
        throw Exception("Engine not ready. "
            "The following errors were encountered:\n" + status, FL_AT);

     calibration = 1;
     FL_LOG("Fuzzy Calibration Complete");
}
    mycount++;
    cmd_vel_pub.publish(msgv);
    ros::spinOnce();
    rate.sleep();
}

     if (calibration == 1){
	if (-(transform.getOrigin().y()+transform2.getOrigin().y()) > -0.45 && -((transform.getOrigin().y()+transform2.getOrigin().y())/2) < 0.45){
		yfinall = -((transform.getOrigin().y()+transform2.getOrigin().y())/2);
        	ytotal->setInputValue(yfinall);
        	engine2->process();
		msgv.linear.x = Vlinear->getOutputValue();
	}
	if (isnan(msgv.linear.x))
	{
	        msgv.linear.x = 0;
	}
	if (-((transform.getOrigin().y()+transform2.getOrigin().y())/2) <= -0.450){
		msgv.linear.x = -1.000;
	}
	if (-((transform.getOrigin().y()+transform2.getOrigin().y())/2) >= 0.450){
		msgv.linear.x = 1.000;
	}
	if (-(transform.getOrigin().x() + transform2.getOrigin().x()) < usrc && -(transform.getOrigin().x() + transform2.getOrigin().x()) > -usrc){
		xfinall = -(transform.getOrigin().x() + transform2.getOrigin().x()); 
        	xtotal->setInputValue(xfinall);
        	engine->process();
		msgv.angular.z = Vangular->getOutputValue();
	}
	if (isnan(msgv.angular.z))
	{
        	msgv.angular.z = 0;
	}
	if (-(transform.getOrigin().x() + transform2.getOrigin().x()) >= usrc){
		msgv.angular.z = 0.600;
	}
	if (-(transform.getOrigin().x() + transform2.getOrigin().x()) <= -usrc){
		msgv.angular.z = -0.600;
	}

	if (transform.getOrigin().x() <= 0.04 && transform2.getOrigin().x() >= -0.04){

		msgv.linear.x = 0;
		msgv.angular.z = 0;
		mycount = 0;
		usrc = 0;
		calibration = 0;
	}

	cmd_vel_pub.publish(msgv);

//	FL_LOG("Va = " << msgv.angular.z << " -> " << "Vl = " << msgv.linear.x);
    ros::spinOnce();
    rate.sleep();
  }
 }
  return 0;
};

