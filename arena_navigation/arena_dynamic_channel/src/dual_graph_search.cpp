#include "arena_dynamic_channel/dual_graph_search.h"

DualGraph::~DualGraph()
{
	for (int i = 0; i < allocate_num_; i++)
  	{
    	delete dual_node_pool_[i];
  	}
    cout<<"DELETE"<<endl;

}

void DualGraph::init(ros::NodeHandle & nh){
	/* init obstacle provider nodes */
	node_=nh;
    ros::master::V_TopicInfo topic_infos;
    ros::master::getTopics(topic_infos);
    std::string str1="obs_dynamic";//"odom_walker";//

	// init vector size
	obs_provider_nodes_.reserve(100);

	// add obs_provider_node to vector
    for (ros::master::V_TopicInfo::iterator it = topic_infos.begin() ; it != topic_infos.end(); it++)
    {
        const ros::master::TopicInfo& info = *it;
        
        if (info.name.find(str1) != std::string::npos) 
        {
            std::cout << "topic_" << it - topic_infos.begin() << ": " << info.name << std::endl;
			obs_provider_nodes_.emplace_back(std::make_shared<ObstacleNode>(node_,info.name));
        }

    }

	/* init delaunay triangulation */
	dt_.reset(new Fade_2D);

	/* init param */
	robot_avg_vel_=1.0; 		//avg_vel of robot m/s
	robot_max_vel_=3.0; 		//avg_vel of robot m/s
	radius_robot_=0.5;
	radius_obs_=0.5;
	have_odom_=false;

	/* map */
	grid_map_.reset(new GridMap);
  	grid_map_->initMap(node_);
	grid_map_->getRegion(occ_map_origin_, occ_map_size_2d_);
	
	resolution_=0.1;								// 0.1 meter/cell
	inv_resolution_=1.0/resolution_;

	time_resolution_=0.1;  							// 0.1 sec
	inv_time_resolution_ = 1.0 / time_resolution_;

	/* search */
	tie_breaker_ = 1.0 + 1.0 / 10000;
	check_num_=20;
	use_node_num_ = 0;
  	iter_num_ = 0;

	// pre-allocated node
	allocate_num_=10000;
	dual_node_pool_.resize(allocate_num_);
  	for (int i = 0; i < allocate_num_; i++)
  	{
    	dual_node_pool_[i] = new DualNode;
  	}

	/* ros communication */
	ros::NodeHandle public_nh;

  	// subscriber
  	goal_sub_ =public_nh.subscribe("goal", 1, &DualGraph::goalCallback,this);
  	odom_sub_ = public_nh.subscribe("odom", 1, &DualGraph::odomCallback, this);

	// publisher
	vis_triangle_pub_= public_nh.advertise<visualization_msgs::Marker>("vis_triangle", 20);
	vis_goal_pub_ =	public_nh.advertise<visualization_msgs::Marker>("vis_goal", 20);
	
	std::cout<<"debug4---------------------"<<std::endl;
	/* init time event timer */
	update_timer_=node_.createTimer(ros::Duration(0.01), &DualGraph::UpdateCallback, this);  // shouldn't use different ros::NodeHandle inside the callback as timer's node handler

	
}

void DualGraph::odomCallback(const nav_msgs::OdometryConstPtr& msg){
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;

  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;

  have_odom_ = true;
}

void DualGraph::goalCallback(const geometry_msgs::PoseStampedPtr& msg){
  if(have_odom_==false) return;

  // end pt
  end_pt_(0) = msg->pose.position.x;    
  end_pt_(1) = msg->pose.position.y;
  
  // vis goal
  std::cout << "Goal set!" << std::endl;
  vector<Eigen::Vector2d> point_set;
  point_set.push_back(end_pt_);
  visualizePoints(point_set,0.5,Eigen::Vector4d(1, 1, 1, 1.0),vis_goal_pub_);

}

void DualGraph::resetGraph(){
	/* init a new Fade2D ptr */
    Fade_2D *dt_temp=new Fade_2D();
	start_pt_=odom_pos_;
	start_vel_=odom_vel_;

	/* insert Points */
	//dt_temp->insert(Point2(start_pt_(0),start_pt_(1)));
	double local_bound_time=5.0; // 3 sec
	Eigen::Vector2d corner_min=start_pt_-Eigen::Vector2d::Ones()*robot_avg_vel_*local_bound_time;
	Eigen::Vector2d corner_max=start_pt_+Eigen::Vector2d::Ones()*robot_avg_vel_*local_bound_time;
	boundPosition(corner_min);
	boundPosition(corner_max);
	Point2 c1=Point2(corner_min(0),corner_min(1));
	Point2 c2=Point2(corner_min(0),corner_max(1));
	Point2 c3=Point2(corner_max(0),corner_min(1));
	Point2 c4=Point2(corner_max(0),corner_max(1));
	//c1.setCustomIndex(100);
	//c2.setCustomIndex(101);
	//c3.setCustomIndex(102);
	//c4.setCustomIndex(103);
	dt_temp->insert(c1);
	dt_temp->insert(c2);
	dt_temp->insert(c3);
	dt_temp->insert(c4);
	

	/* reset obs_state_set_*/
	obs_state_set_.clear();

	/* build a time-space trianglulation graph */
	int custom_index=0;
	for (std::vector<ObstacleNode::Ptr>::iterator it = obs_provider_nodes_.begin() ; it != obs_provider_nodes_.end(); it++)
    {
        const ObstacleNode::Ptr & obs_info = *it;
		// calculate arriving time according to relative velocity
		Eigen::Vector2d obs_pos=obs_info->getPosition();
		Eigen::Vector2d obs_vel=obs_info->getVelocity();
		Eigen::Vector2d dist=obs_pos-start_pt_;  							// vec_Robot2Obstacle
		dist(0)=dist(0)==0.0?0.0001:dist(0);
		dist(1)=dist(1)==0.0?0.0001:dist(1);
		double obs_vel_project=obs_vel.dot(-dist)/dist.norm();

		double arriving_time=dist.norm()/(robot_avg_vel_+obs_vel_project); 	// assume robot_vel target at the obstacle pos at t=t0
		arriving_time=std::max(arriving_time,0.0);							// if arriving_time<0, means can collid immediately

		/*  */
		//std::cout<<"*********************************"<<std::endl;
		//std::pair<double, double> t_collide=computeCollisionTime(start_pt_,start_vel_,obs_pos,obs_vel,1.5);
		//std::cout<<"t_collide"<<t_collide.first<<"   "<<t_collide.second<<std::endl;
		
		if(arriving_time>3.0)
		{
			continue;
		}
		//std::cout<<"---------------------"<<std::endl;
		//std::cout<<"obs_vel_project"<<obs_vel_project<<std::endl;
		//std::cout<<"arriving_time"<<arriving_time<<std::endl;

		// make an obs_state
		ObstacleState::Ptr obs_state_ptr = std::make_shared<ObstacleState>(obs_info->getPosition(),obs_info->getVelocity(),arriving_time,custom_index);
		
		// push back obs_state
		obs_state_set_.push_back(obs_state_ptr);

		dt_temp->insert(obs_state_ptr->point2_pos_t);
		custom_index++;
		
	}

	//std::cout << "a---------------------------------" << std::endl;
	//std::cout<<"size="<<obs_state_set_.size()<<std::endl;
    //printf("%d\n", obs_state_set_.back().use_count());
	//std::cout<<"debug54---------------------"<<std::endl;

	/* reset dt_ */
	dt_.reset(dt_temp);
	//std::cout<<"debug55---------------------"<<std::endl;
}

void DualGraph::searchChannel(){
	// reset time origin
	time_origin_=0.0;  //ros::Time::now().toSec();
	
	// reset start & end point
	start_pt_=odom_pos_;
	start_vel_=odom_vel_;

	end_pt_=end_pt_;

	DualNodePtr cur_node = dual_node_pool_[0];
	cur_node->parent = NULL;
	cur_node->pos = start_pt_;
	cur_node->vel = start_vel_;
	cur_node->index = posToIndex(start_pt_);
	cur_node->g_score = 0.0;


	// reset end node

	// reset dualGraph
	resetGraph(); 


}



double DualGraph::computeArrivingTime(Eigen::Vector2d curr_pos,Eigen::Vector2d curr_vel,Eigen::Vector2d next_pos,Eigen::Vector2d next_vel)
{	// the time arriving the next_pos from curr_pos, regarding only robot itself
	double t=(curr_pos-next_pos).norm()/robot_avg_vel_;
	return t;
}

std::pair<double, double> DualGraph::computeCollisionTime(Eigen::Vector2d pos1,Eigen::Vector2d vel1, Eigen::Vector2d pos2, Eigen::Vector2d vel2, double dist_thresh){
	// calculate polynomial 
	Eigen::Vector2d dp=pos1-pos2;
	Eigen::Vector2d dv=vel1-vel2;
	double a2,a1,a0;
	a2= dv.dot(dv);
	a1=2*dp.dot(dv);
	a0=dp.dot(dp)-dist_thresh*dist_thresh;

	// calculate companion matrix of the polynomial
	Eigen::Matrix2d Compan;
    Compan<<-a1/a2	,	-a0/a2,
			1.0		,  	0.0;

	// solve the eigenvalue of companion matrix as root
	Eigen::EigenSolver<Eigen::MatrixXd> solver(Compan);
	auto T_vals=solver.eigenvalues();
	
	// select root
	double t1, t2;
	if(std::abs(T_vals(0).imag())>0.00001){
		// if has imag value, then never collide
		t1=t2=-1.0;
	}else{
		// have solution, between t1 and t2
		if(T_vals(0).real()>0){
			t1=std::max(T_vals(0).real(),0.0);
			t2=std::max(T_vals(1).real(),0.0);
		}else{
			// if t1<0,t2<0 or t1=t2=nan
			t1=t2=-1.0;
		}
		
	}

    return std::make_pair(t1, t2);
}

bool DualGraph::isDynamicSafe(DualNodePtr curr_node, Eigen::Vector2d next_pos, double & min_to_collide_time){
	// compute jounney time range from t_curr
	double t_curr=curr_node->time;
	
	// check collision for every obstacle node
	double d_thresh=2*(radius_robot_+radius_obs_);

	// robot pos & vel from t_curr
	Eigen::Vector2d robot_pos_t=curr_node->pos;
	Eigen::Vector2d robot_vel_t= (next_pos - curr_node->pos)/(next_pos - curr_node->pos).norm()*robot_avg_vel_;

	// durations
	double dur_arrive=(next_pos-curr_node->pos).norm()/robot_avg_vel_;
	min_to_collide_time=10000;
	for(size_t i=0;i<obs_state_set_.size();++i){
		ObstacleState::Ptr obs;
		// obs pos & vel from t_curr
		Eigen::Vector2d obs_pos_t = t_curr * obs->vel + obs->pos;
		Eigen::Vector2d obs_vel_t = obs->vel;
		
		std::pair<double, double> dur_collide=computeCollisionTime(obs_pos_t,obs_vel_t,robot_pos_t,robot_vel_t,d_thresh);
		if(0.0>dur_collide.first || dur_arrive <dur_collide.second )
		{
			if(dur_collide.second>0.0){
				min_to_collide_time=std::min(min_to_collide_time,(dur_collide.second-dur_arrive));
			}
		}else{
			return false;
		}
	}
	return true;
}

bool DualGraph::isGateFeasible(ObstacleState::Ptr obs1, ObstacleState::Ptr obs2, double t_arrive){
	// calculate from t_origin
	double d_thresh=(radius_obs_+radius_obs_)*2;
	std::pair<double, double> t_collide=computeCollisionTime(obs1->pos,obs1->vel,obs2->pos,obs2->vel,d_thresh);
	double t_collide_max=t_collide.first;
	double t_collide_min=t_collide.second;
	
	// check if t_arrive is within colliding time range
	if(t_arrive<t_collide_max && t_arrive>t_collide_min){
		return false;
	}else{
		return true;
	}
}

double DualGraph::getHeuristic(Eigen::Vector2d curr_pos,Eigen::Vector2d end_pos){
	return (curr_pos-end_pos).norm()/robot_avg_vel_;
}

void DualGraph::UpdateCallback(const ros::TimerEvent&){
	
	ros::WallTime t1, t2;
  	t1 = ros::WallTime::now();
	
	
	resetGraph();
	
	t2 = ros::WallTime::now();
	/* record time */
	dc_time_ += (t2 - t1).toSec();
	max_dc_time_ = std::max(max_dc_time_, (t2 - t1).toSec());
  	dc_update_num_++;
	
	bool show_time=false;
  	if (show_time)
	{
		ROS_WARN("DC: cur t = %lf, avg t = %lf, max t = %lf", 
				(t2 - t1).toSec(),
             	dc_time_/ dc_update_num_,
				max_dc_time_);
	}
  	
	/* publish Graph */
	publishVisGraph();

}



/* --------------------visualization---------------------------- */
void DualGraph::visualizePoints(const vector<Eigen::Vector2d>& point_set, double pt_size, const Eigen::Vector4d& color, const ros::Publisher & pub) {
  
  visualization_msgs::Marker mk;
  mk.header.frame_id = "map";
  mk.header.stamp    = {};//ros::Time::now();
  mk.type            = visualization_msgs::Marker::SPHERE_LIST;
  mk.action          = visualization_msgs::Marker::DELETE;
  //mk.id              = id;

  mk.action             = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = pt_size;
  mk.scale.y = pt_size;
  mk.scale.z = pt_size;

  geometry_msgs::Point pt;
  for (unsigned int i = 0; i < point_set.size(); i++) {
    pt.x = point_set[i](0);
    pt.y = point_set[i](1);
    pt.z = 0.0;
    mk.points.push_back(pt);
  }
  pub.publish(mk);
  ros::Duration(0.001).sleep();
}

void DualGraph::visualizeLines(const std::vector<std::pair<Eigen::Vector2d,Eigen::Vector2d>> & ptr_pair_sets, double pt_size, const Eigen::Vector4d& color, const ros::Publisher & pub){
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "/map";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "lines";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 2;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = pt_size;
    line_list.color.r = color(0);
    line_list.color.g = color(1);
    line_list.color.b = color(2);
    line_list.color.a = color(3);

    geometry_msgs::Point pt1,pt2;
    for (unsigned int i = 0; i < ptr_pair_sets.size(); i++) {
        pt1.x=ptr_pair_sets[i].first(0);
        pt1.y=ptr_pair_sets[i].first(1);
        pt1.z=0.0;

        pt2.x=ptr_pair_sets[i].second(0);
        pt2.y=ptr_pair_sets[i].second(1);
        pt2.z=0.0;

        line_list.points.push_back(pt1);
        line_list.points.push_back(pt2);
    }

    pub.publish(line_list);
    //ROS_INFO("vis once");
}

void DualGraph::publishVisGraph(){

	std::vector<std::pair<Eigen::Vector2d,Eigen::Vector2d>> line_sets;

	std::vector<Triangle2*> vAllDelaunayTriangles;
	dt_->getTrianglePointers(vAllDelaunayTriangles);
	for(std::vector<Triangle2*>::iterator it=vAllDelaunayTriangles.begin();it!=vAllDelaunayTriangles.end();++it)
	{
		Triangle2* pT(*it);
		// An alternative (just to show how to access the vertices) would be:
		Point2* p0=pT->getCorner(0);
		Point2* p1=pT->getCorner(1);
		Point2* p2=pT->getCorner(2);

		std::pair<Eigen::Vector2d,Eigen::Vector2d> line1(Eigen::Vector2d(p0->x(),p0->y()),Eigen::Vector2d(p1->x(),p1->y()));
		std::pair<Eigen::Vector2d,Eigen::Vector2d> line2(Eigen::Vector2d(p0->x(),p0->y()),Eigen::Vector2d(p2->x(),p2->y()));
		std::pair<Eigen::Vector2d,Eigen::Vector2d> line3(Eigen::Vector2d(p2->x(),p2->y()),Eigen::Vector2d(p1->x(),p1->y()));
		line_sets.push_back(line1);
		line_sets.push_back(line2);
		line_sets.push_back(line3);
	} 

	visualizeLines(line_sets,0.2,Eigen::Vector4d(0, 1, 1, 1.0),vis_triangle_pub_);
}


/* --------------------main---------------------------- */
int main(int argc, char** argv)
{

	ros::init(argc, argv, "obstacle2");
    ros::NodeHandle nh("");
    DualGraph::Ptr dual_graph;

    dual_graph.reset(new DualGraph);
    dual_graph->init(nh);
	ros::spin();

	/* ros::Rate r=ros::Rate(10);
	while(ros::ok()){
		//dual_graph->resetGraph();
		//dual_graph->publishVisGraph();
		ros::spinOnce();
		r.sleep();
	} */




    // std::vector<Eigen::Vector2d> pt_set;
    // pt_set.push_back(Eigen::Vector2d(-10,-10));
    // pt_set.push_back(Eigen::Vector2d(+10,+10));
    // pt_set.push_back(Eigen::Vector2d(-5,-7));
    // pt_set.push_back(Eigen::Vector2d(-5,-3));
    // pt_set.push_back(Eigen::Vector2d(5,7));
    // pt_set.push_back(Eigen::Vector2d(5,3));

    // gp->resetCDT(pt_set,30.0);
    // cout<<"abc"<<endl;

}








// void DualGraph::resetCDT(const std::vector<Eigen::Vector2d> &pt_set, const double thresh ){
//     /* init a new Fade2D ptr */
//     Fade_2D *dt_temp=new Fade_2D();
    

// 	/* add pts to dt_temp */
// 	// Insert 4 bounding box points
//     dt_temp->insert(Point2(0,0));
// 	dt_temp->insert(Point2(+100,0));
// 	dt_temp->insert(Point2(100,50));
// 	dt_temp->insert(Point2(0,50));

//     input_points_set_.clear();
//     input_points_set_=pt_set; //save new input points

//     for(unsigned int i=0; i<pt_set.size();++i)
//     {   
//         dt_temp->insert(Point2(pt_set[i](0),pt_set[i](1)));
//     }
    

//     /* select constraint seg from tiangles */
//     std::vector<Segment2> constrained_segments;
//     // check all existed DTs 
//     std::vector<Triangle2*> vAllDelaunayTriangles;
// 	dt_temp->getTrianglePointers(vAllDelaunayTriangles);
// 	for(std::vector<Triangle2*>::iterator it=vAllDelaunayTriangles.begin();it!=vAllDelaunayTriangles.end();++it)
// 	{
// 		Triangle2* pT(*it);

//         for(int j=0;j<2;++j)
//         {   
//             int first=j, second=(j+1)>2?0:(j+1);
            
//             Segment2 s=Segment2(*pT->getCorner(first),*pT->getCorner(second)); // Point2 * 	getCorner (const int ith) const
//             cout<<"edge length:"<<s.getSqLen2D()<<endl;
//             // if Segment length < thresh, then seen it as constrained segment
//             if(s.getSqLen2D()<thresh)
//             {
//                 constrained_segments.push_back(s);
//             }
//         }
// 	}
//     // set constained edge
//     if(!constrained_segments.empty())
//     {
//         ConstraintGraph2* pCG=dt_temp->createConstraint(constrained_segments,CIS_CONSTRAINED_DELAUNAY);
        
//         double minLen(0.1);
// 	    pCG->makeDelaunay(minLen);
//     }

//     // reset dt_
//     dt_.reset(dt_temp);
//     cout<<"-----------show"<<endl;
//     Visualizer2 * pvis=new Visualizer2("MyExample.ps") ;
//     dt_->show(pvis);
//     visualize_vertices(pvis);//dt_->show("My_example_makeDelaunay_1.ps",true);
//     pvis->writeFile();
//     cout<<"-----------finished"<<endl;
// }

// void DualGraph::visualize_vertices( Visualizer2 * vis){
//     // init vis 
//     //Visualizer2 vis(draw_name.c_str());
    
//     // define colors
// 	Color cBlack(CBLACK);
// 	Color cBlue(CBLUE);
// 	Color cRed(CRED);
// 	Color cPurple(CPURPLE);
//     Color cGreenFill(CGREEN,.001f,true);
//     // vertice circle radius
//     double sqRadius(1*1);

// 	// * 2 *   Get and draw the vertices with their custom index
// 	/* std::vector<Point2*> vAllPoints;
// 	dt_->getVertexPointers(vAllPoints);
// 	std::cout<<"vAllPoints.size()="<<vAllPoints.size()<<std::endl;
// 	for(std::vector<Point2*>::iterator it(vAllPoints.begin());it!=vAllPoints.end();++it)
// 	{
// 		Point2* currentPoint(*it);

//         // add label
// 		int customIndex(currentPoint->getCustomIndex());
//         cout<<"idx="<<Point2(currentPoint->x(),currentPoint->y())<<endl;
// 		std::string text=toString(customIndex);
// 		vis.addObject(Label(*currentPoint,text.c_str(),true,12),cPurple);
//         // add circle
//         Circle2 circ0(currentPoint->x(),currentPoint->y(),sqRadius);
//         vis.addObject(circ0,cGreenFill);
// 	} */
//     for( unsigned int i=0;i<input_points_set_.size();++i)
//     {
//         Circle2 circ0(input_points_set_[i](0),input_points_set_[i](1),sqRadius);
//         vis->addObject(circ0,cGreenFill);
//     }


//     // * 3 *   Get and draw the triangles
// 	/* std::vector<Triangle2*> vAllDelaunayTriangles;
// 	dt_->getTrianglePointers(vAllDelaunayTriangles);
// 	for(std::vector<Triangle2*>::iterator it=vAllDelaunayTriangles.begin();it!=vAllDelaunayTriangles.end();++it)
// 	{
// 		Triangle2* pT(*it);
// 		vis->addObject(*pT,cBlack);

// 		// An alternative (just to show how to access the vertices) would be:
// 		//Point2* p0=pT->getCorner(0);
// 		//Point2* p1=pT->getCorner(1);
// 		//Point2* p2=pT->getCorner(2);
// 		//vis.addObject(Segment2(*p0,*p1),cBlack);
// 		//vis.addObject(Segment2(*p1,*p2),cBlack);
// 		//vis.addObject(Segment2(*p2,*p0),cBlack);
// 	} */
// }






















/*
int main()
{
	std::cout<<"* Example4: Zones - Defines zones in different ways"<<std::endl;
	std::cout<<"  0) area inside a ConstraintGraph2"<<std::endl;
	std::cout<<"  1) area outside a ConstraintGraph2"<<std::endl;
	std::cout<<"  2) area grown from a seed point"<<std::endl;
	std::cout<<"  3) global (all triangles)"<<std::endl;
	std::cout<<"  4) area of arbitrary triangles\n"<<std::endl;

	// * 1 *   Insert 4 bounding box points
	Fade_2D dt;
	dt.insert(Point2(0,0));
	dt.insert(Point2(+100,0));
	dt.insert(Point2(100,50));
	dt.insert(Point2(0,50));

	// * 2 *   Create points on two circles
	std::vector<Point2> vCircle0;
	std::vector<Point2> vCircle1;
	int numPoints(8);
	double radius(22);
	//generateCircle( num,centerX,centerY,radiusX,radiusY,vOut);
	generateCircle(numPoints,25.0,25.0,radius,radius,vCircle0);
	generateCircle(numPoints,75.0,25.0,radius,radius,vCircle1);

	// * 3 *   Create segments
	std::vector<Segment2> vSegments0;
	std::vector<Segment2> vSegments1;
	for(int i=0;i<numPoints;++i)
	{
		Segment2 seg0(vCircle0[i],vCircle0[(i+1)%numPoints]);
		Segment2 seg1(vCircle1[i],vCircle1[(i+1)%numPoints]);
		vSegments0.push_back(seg0);
		vSegments1.push_back(seg1);
	}

	// * 4 *   Insert the segments as constraint graphs
	ConstraintGraph2* pCG0=dt.createConstraint(vSegments0,CIS_CONSTRAINED_DELAUNAY);
	ConstraintGraph2* pCG1=dt.createConstraint(vSegments1,CIS_CONSTRAINED_DELAUNAY);

	// * 5 *  Visualize
	dt.show("example4_constraints.ps",true);

	// Verify: pCG0 and pCG1 must be closed (GASSEX is defined in "someTools.h")
	GASSEX(pCG0->isPolygon());
	GASSEX(pCG1->isPolygon());

	// * 6 *   Create Zone2 objects in different ways, then retrieve the
	//    triangles and visualize the zones

	// + Zone inside pCG0:
	Zone2* pZoneInside(dt.createZone(pCG0,ZL_INSIDE));
	pZoneInside->show("example4_zoneInside.ps",true,true); // all triangles=true, constraints=true

	// + Zone outside pCG0:
	Zone2* pZoneOutside(dt.createZone(pCG0,ZL_OUTSIDE));
	pZoneOutside->show("example4_zoneOutside.ps",true,true);

	// + Zone grown from a seed-point, the growing stops at the
	//   edges of the specified ConstraintGraph2 objects
	std::vector<ConstraintGraph2*> vCG;
	vCG.push_back(pCG0);
	vCG.push_back(pCG1);
	Point2 seedPoint(5.0,5.0); // Point near the lower left corner
	Zone2* pZoneGrow(dt.createZone(vCG,ZL_GROW,seedPoint));
	pZoneGrow->show("example4_zoneGrow.ps",true,true);

	// + Global zone (all triangles):
	Zone2* pZoneGlobal(dt.createZone(NULL,ZL_GLOBAL));
	pZoneGlobal->show("example4_zoneGlobal.ps",true,true);

	// + Zone defined by specific triangles
	std::vector<Triangle2*> vT;
	dt.getTrianglePointers(vT);
	vT.resize(vT.size()/2);
	Zone2* pZoneRandomTriangles(dt.createZone(vT));
	pZoneRandomTriangles->show("example4_zoneFromTriangles.ps",true,true);
	return 0;
}

*/



/* 

int main()
{
	std::cout<<"Example3: Constraints - Enforce constraint edges\n";

	// 1) Generate some input points
	std::vector<Point2> vInputPoints;
	vInputPoints.push_back(Point2(-100,-100));
	vInputPoints.push_back(Point2(+100,+100));
	vInputPoints.push_back(Point2(-50,-70));
	vInputPoints.push_back(Point2(-50,-30));
	vInputPoints.push_back(Point2(50,70));
	vInputPoints.push_back(Point2(50,30));

	// 2) Triangulate the points and show
	Fade_2D dt;
	dt.insert(vInputPoints);
	dt.show("example3_noConstraints.ps",true);

	// 3) Prepare a vector of one or more edges to be enforced
	std::vector<Segment2> vSegments;
	vSegments.push_back(Segment2(vInputPoints[0],vInputPoints[1]));

	// 4) Insert the Constraint Segments
	// Use always CIS_CONSTRAINED_DELAUNAY. This strategy does not
	// subdivide the constraint segments except when they intersect
	// an existing vertex or another constraint segment.
	// Other insertion strategies exist also but they are deprecated
	// and only kept for backwards compatibility. Their behavior is
	// perfectly replaced by fast and reliable methods like the
	// functions ConstraintGraph2::makeDelaunay() and Fade_2D::drape().
	ConstraintGraph2* pCG=dt.createConstraint(vSegments,CIS_CONSTRAINED_DELAUNAY);
	dt.show("example3_withConstraints.ps",true);

	// 5) makeDelaunay() - an optional function call to subdivide a
	// ConstraintGraph in order to achieve better shaped triangles.
	// Segments smaller than $minLen are not subdivided. This parameter
	// is thought to prevent excessive subdivision in narrow settings.
	double minLen(0.1);
	pCG->makeDelaunay(minLen);
	dt.show("example3_makeDelaunay.ps",true);

    

	return 0;
}

 */
/*
// Prototypes
void manualDraw(Fade_2D& dt);
void moreDraw();

 

int main()
{
	std::cout<<"* Example2: Access elements of a triangulation\n";

	// * 1 *   Create points on a circle
	std::vector<Point2> vPoints;
	int numPoints(6);
	double centerX(5),centerY(5),radiusX(5),radiusY(5);
	generateCircle( numPoints,centerX,centerY,radiusX,radiusY,vPoints);
	vPoints.push_back(Point2(centerX,centerY)); // Add the center point

	// * 2 *   Optional step: You can add custom indices to relate
	//         the points to your own data structures. But you do
	//         not need to use this feature.
	int myStartIndex(77); // An arbitrary value
	for(size_t i=0;i<vPoints.size();++i)
	{
		vPoints[i].setCustomIndex(myStartIndex++);
	}

	// * 3 *   Insert the vertices
	Fade_2D dt; // The Delaunay triangulation
	std::vector<Point2*> vVertexHandles; // Pointers to the vertices inside Fade
	dt.insert(vPoints,vVertexHandles); // Fastest method: insert all points at once

	// * 4 *   Draw the triangulation using a ready-made function
	dt.show("example2_triangulation.ps");

	// * 5 *   Manual draw (to demonstrate access to the elements)
	manualDraw(dt);

	// * 6 *   More drawing examples using the Visualizer2 class
	moreDraw();

	return 0;
}













// Manual visualization of a triangulation. Note: This can also be
// done with the Fade_2D::show() function but the goal here is also
// to demonstrate how to access the elements of a triangulation.
void manualDraw(Fade_2D& dt)
{
	// * 1 *   Create a postscript visualizer and define some colors
	Visualizer2 vis("example2_manualDraw.ps");
	Color cBlack(CBLACK);
	Color cBlue(CBLUE);
	Color cRed(CRED);
	Color cPurple(CPURPLE);
    Color cGreenFill(CGREEN,.001f,true);
	vis.addObject(Label(Point2(1.5,12.4),"BLACK: triangle edges",false,15),cBlack);
	vis.addObject(Label(Point2(1.5,11.6),"RED: neighbor-triangle labels",false,15),cRed);
	vis.addObject(Label(Point2(1.5,10.8),"BLUE: intra-triangle-indices",false,15),cBlue);
	vis.addObject(Label(Point2(1.5,10.0),"PURPLE: custom vertex indices",false,15),cPurple);

    double sqRadius(0.1*0.1);
	

	// * 2 *   Get and draw the vertices with their custom index
	std::vector<Point2*> vAllPoints;
	dt.getVertexPointers(vAllPoints);
	std::cout<<"vAllPoints.size()="<<vAllPoints.size()<<std::endl;
	for(std::vector<Point2*>::iterator it(vAllPoints.begin());it!=vAllPoints.end();++it)
	{
		Point2* currentPoint(*it);
		int customIndex(currentPoint->getCustomIndex());
        cout<<"idx="<<Point2(currentPoint->x(),currentPoint->y())<<endl;
		std::string text=toString(customIndex);
		vis.addObject(Label(*currentPoint,text.c_str(),true,12),cPurple);
        Circle2 circ0(currentPoint->x(),currentPoint->y(),sqRadius);
        vis.addObject(circ0,cGreenFill);

	}

	// * 3 *   Get and draw the triangles
	std::vector<Triangle2*> vAllDelaunayTriangles;
	dt.getTrianglePointers(vAllDelaunayTriangles);
	for(std::vector<Triangle2*>::iterator it=vAllDelaunayTriangles.begin();it!=vAllDelaunayTriangles.end();++it)
	{
		Triangle2* pT(*it);
		vis.addObject(*pT,cBlack);

		// An alternative (just to show how to access the vertices) would be:
		//Point2* p0=pT->getCorner(0);
		//Point2* p1=pT->getCorner(1);
		//Point2* p2=pT->getCorner(2);
		//vis.addObject(Segment2(*p0,*p1),cBlack);
		//vis.addObject(Segment2(*p1,*p2),cBlack);
		//vis.addObject(Segment2(*p2,*p0),cBlack);
	}

	// * 4 *   Choose one triangle and color it green
	Triangle2* pT(vAllDelaunayTriangles[0]);
	//Color cGreenFill(CPALEGREEN,0.001f,true);
	vis.addObject(*pT,cGreenFill);

	// * 5 *   The corners of pT can be accessed through the so called intra-
	// triangle-indices 0,1,2. They are counterclockwise oriented (CCW).
	// Let's write intra-triangle-index labels beside the corners.
	for(int intraTriangleIndex=0;intraTriangleIndex<3;++intraTriangleIndex)
	{
		Point2* pCorner(pT->getCorner(intraTriangleIndex));
		std::string text("idx");//"\nidx="+toString(intraTriangleIndex)
		vis.addObject(Label(*pCorner,text.c_str(),true,15),cBlue);
	}

	// * 6 *   Each triangle has three neighbor triangles (or NULL
	// pointers at border edges). They are accessed through the
	// intra-triangle-indices. The i'th opposite triangle of pT is
	// the one that is opposite to the i'th vertex. Let's draw that:
	Label label_pT(pT->getBarycenter()," pT",true,15);
	vis.addObject(label_pT,cBlue);
	for(int intraTriangleIndex=0;intraTriangleIndex<3;++intraTriangleIndex)
	{
		Triangle2* pNeigT(pT->getOppositeTriangle(intraTriangleIndex));
		if(pNeigT==NULL) continue; // No adjacent triangle at this edge

		// Compute the barycenter and write a label there
		Point2 barycenter(pNeigT->getBarycenter());
		std::string text(" =pT->getOppositeTriangle("+toString(intraTriangleIndex)+")");
		Label neigLabel(barycenter,text.c_str(),true,15);
		vis.addObject(neigLabel,cRed);
	}

	// Write the postscript file to disk
	vis.writeFile();
}

// More examples that demonstrate the use of the Visualizer2 class
void moreDraw()
{
	// * 1 *   Defining color
	// Define red by values (red,green,blue,linewidth,bFill)
	Color cRed(1,0,0,0.001,false);
	// Or simply use a color name and the defaults linewidth=0.001 and bFill=false
	Color cBlue(CBLUE);
	Color cGreen(CGREEN);
	Color cPurple(CPURPLE);
	// Specify a bolt green color (linewidth=1.0) and another one with
	// bFill=true to fill the area of an object which is drawn using
	// that color. In case that a segment is drawn with bFill=true,
	// marks for its endpoints are added.
	Color cGreenBolt(CGREEN,1.0,false);
	Color cGreenFill(CGREEN,.001f,true);

	// Postscript writer
	Visualizer2 vis("example2_moreDraw.ps");

	// Create a label and add it to the Visualizer
	Label headLabel(Point2(20,60),	// Position
					"This is the Fade2D\nPostscript Visualizer",
					false,			// Don't write an x-mark
					30);			// Font size
	vis.addObject(headLabel,cRed); 	// Add the label

	// Add a row of points
	for(int x=0;x<100;x+=5)
	{
		Point2 p(x,50);
		vis.addObject(p,cPurple); // Add the point
	}

	// Draw a segment with cGreen
	Point2 p0(0,0);
	Point2 p1(100,0);
	Segment2 seg0(p0,p1);
	vis.addObject(seg0,cGreen); // Add the segment

	// Draw a segment with cGreenFill (with marks at the endpoints)
	Point2 p2(0,10);
	Point2 p3(100,10);
	Segment2 seg1(p2,p3);
	vis.addObject(seg1,cGreenFill); // Add the segment

	// Draw a segment with cGreenBolt
	Point2 p4(0,20);
	Point2 p5(100,20);
	Segment2 seg2(p4,p5);
	vis.addObject(seg2,cGreenBolt); // Add the segment

	// Draw labels
	Label lab0(Point2(20,2),"Segment in cGreen",false,15);
	vis.addObject(lab0,cBlue); // Add the label

	Label lab1(Point2(20,12),"Segment in cGreenFill",false,15);
	vis.addObject(lab1,cBlue); // Add the label

	Label lab2(Point2(20,22),"Segment in cGreenBolt",false,15);
	vis.addObject(lab2,cBlue); // Add the label

	// Add three circles with radius=4 (squared radius 16)
	double sqRadius(4.0*4.0);
	Circle2 circ0(50,40,sqRadius);
	Circle2 circ1(60,40,sqRadius);
	Circle2 circ2(70,40,sqRadius);
	vis.addObject(circ0,cGreen);	// Add the circle
	vis.addObject(circ1,cGreenBolt);// Add the circle
	vis.addObject(circ2,cGreenFill);// Add the circle

	// The postscript file is only written when writeFile() is called.
	vis.writeFile();
}
 */

/* 
int main()
{
	std::cout<<"* Example1: Benchmark\n";
	std::cout<<"* Measures the (single-/multithreaded) triangulation time\n\n";

	// * 1 *   Create a Fade object.
	Fade_2D dt;
	int numUsedCPU=dt.setNumCPU(0); // 0 means: autodetect
	cout<<"Number of CPU cores: "<<numUsedCPU<<endl;

	// * 2 *   Set up how many points to test
	std::vector<std::pair<std::string,int> > vNumPoints;
	vNumPoints.push_back(make_pair("numPoints: 1k",1000));
	vNumPoints.push_back(make_pair("numPoints: 50k",50000));
	vNumPoints.push_back(make_pair("numPoints: 100k",100000));
	vNumPoints.push_back(make_pair("numPoints: 500k",500000));
	vNumPoints.push_back(make_pair("numPoints: 1 mio",1000000));
	vNumPoints.push_back(make_pair("numPoints: 10 mio",10000000));
	//vNumPoints.push_back(make_pair("numPoints: 50 mio (10.7 GB)",50000000));
	//vNumPoints.push_back(make_pair("numPoints: 100 mio (21.3 GB)",100000000));
	//vNumPoints.push_back(make_pair("numPoints: 250 mio (53 GB)",250000000));

	// * 3 *   Test
	for(size_t i=0;i<vNumPoints.size();++i)
	{
		std::string label(vNumPoints[i].first);
		int numPoints(vNumPoints[i].second);

		// Prepare a vector of random points
		vector<Point2> vInPoints;
		generateRandomPoints(numPoints,0,100,vInPoints,1);

		// Insert the points and erase afterwards. The total time for
		// triangulation (without destruction time) is measured.
		cout<<"\n"<<label<<": start"<<endl;
		double elapsed=dt.measureTriangulationTime(vInPoints);
		cout<<"Elapsed time: "<<elapsed<<endl<<endl;
	}

	return 0;
}
 */