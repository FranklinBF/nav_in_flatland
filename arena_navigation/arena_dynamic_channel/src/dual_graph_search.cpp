#include "arena_dynamic_channel/dual_graph_search.h"

DualGraph::~DualGraph()
{
    cout<<"DELETE"<<endl;

}

void DualGraph::init(){
    dt_.reset(new Fade_2D);
}

void DualGraph::resetCDT(const std::vector<Eigen::Vector2d> &pt_set, const double thresh ){
    /* init a new Fade2D ptr */
    Fade_2D *dt_temp=new Fade_2D();
    // Insert 4 bounding box points
    dt_temp->insert(Point2(0,0));
	dt_temp->insert(Point2(+100,0));
	dt_temp->insert(Point2(100,50));
	dt_temp->insert(Point2(0,50));
   
    /* add pts to dt_temp */
    input_points_set_.clear();
    input_points_set_=pt_set;

    for(unsigned int i=0; i<pt_set.size();++i)
    {   
        dt_temp->insert(Point2(pt_set[i](0),pt_set[i](1)));
        //input_points_set_.push_back(pt_set[i]);
    }
    

    /* select constraint seg from tiangles */
    std::vector<Segment2> constrained_segments;
    // check all existed DTs 
    std::vector<Triangle2*> vAllDelaunayTriangles;
	dt_temp->getTrianglePointers(vAllDelaunayTriangles);
	for(std::vector<Triangle2*>::iterator it=vAllDelaunayTriangles.begin();it!=vAllDelaunayTriangles.end();++it)
	{
		Triangle2* pT(*it);

        for(int j=0;j<2;++j)
        {   
            int first=j, second=(j+1)>2?0:(j+1);
            
            Segment2 s=Segment2(*pT->getCorner(first),*pT->getCorner(second)); // Point2 * 	getCorner (const int ith) const
            cout<<"edge length:"<<s.getSqLen2D()<<endl;
            // if Segment length < thresh, then seen it as constrained segment
            if(s.getSqLen2D()<thresh)
            {
                constrained_segments.push_back(s);
            }
        }
	}
    // set constained edge
    if(!constrained_segments.empty())
    {
        ConstraintGraph2* pCG=dt_temp->createConstraint(constrained_segments,CIS_CONSTRAINED_DELAUNAY);
        
        double minLen(0.1);
	    pCG->makeDelaunay(minLen);
    }

    // reset dt_
    dt_.reset(dt_temp);
    cout<<"-----------show"<<endl;
    Visualizer2 * pvis=new Visualizer2("MyExample.ps") ;
    dt_->show(pvis);
    visualize_vertices(pvis);//dt_->show("My_example_makeDelaunay_1.ps",true);
    pvis->writeFile();
    cout<<"-----------finished"<<endl;
}

void DualGraph::visualize_vertices( Visualizer2 * vis){
    // init vis 
    //Visualizer2 vis(draw_name.c_str());
    
    // define colors
	Color cBlack(CBLACK);
	Color cBlue(CBLUE);
	Color cRed(CRED);
	Color cPurple(CPURPLE);
    Color cGreenFill(CGREEN,.001f,true);
    // vertice circle radius
    double sqRadius(1*1);

	// * 2 *   Get and draw the vertices with their custom index
	/* std::vector<Point2*> vAllPoints;
	dt_->getVertexPointers(vAllPoints);
	std::cout<<"vAllPoints.size()="<<vAllPoints.size()<<std::endl;
	for(std::vector<Point2*>::iterator it(vAllPoints.begin());it!=vAllPoints.end();++it)
	{
		Point2* currentPoint(*it);

        // add label
		int customIndex(currentPoint->getCustomIndex());
        cout<<"idx="<<Point2(currentPoint->x(),currentPoint->y())<<endl;
		std::string text=toString(customIndex);
		vis.addObject(Label(*currentPoint,text.c_str(),true,12),cPurple);
        // add circle
        Circle2 circ0(currentPoint->x(),currentPoint->y(),sqRadius);
        vis.addObject(circ0,cGreenFill);
	} */
    for( unsigned int i=0;i<input_points_set_.size();++i)
    {
        Circle2 circ0(input_points_set_[i](0),input_points_set_[i](1),sqRadius);
        vis->addObject(circ0,cGreenFill);
    }


    // * 3 *   Get and draw the triangles
	/* std::vector<Triangle2*> vAllDelaunayTriangles;
	dt_->getTrianglePointers(vAllDelaunayTriangles);
	for(std::vector<Triangle2*>::iterator it=vAllDelaunayTriangles.begin();it!=vAllDelaunayTriangles.end();++it)
	{
		Triangle2* pT(*it);
		vis->addObject(*pT,cBlack);

		// An alternative (just to show how to access the vertices) would be:
		//Point2* p0=pT->getCorner(0);
		//Point2* p1=pT->getCorner(1);
		//Point2* p2=pT->getCorner(2);
		//vis.addObject(Segment2(*p0,*p1),cBlack);
		//vis.addObject(Segment2(*p1,*p2),cBlack);
		//vis.addObject(Segment2(*p2,*p0),cBlack);
	} */
}

int main()
{
    DualGraph::Ptr gp;

    gp.reset(new DualGraph);
    gp->init();

    std::vector<Eigen::Vector2d> pt_set;
    pt_set.push_back(Eigen::Vector2d(-10,-10));
    pt_set.push_back(Eigen::Vector2d(+10,+10));
    pt_set.push_back(Eigen::Vector2d(-5,-7));
    pt_set.push_back(Eigen::Vector2d(-5,-3));
    pt_set.push_back(Eigen::Vector2d(5,7));
    pt_set.push_back(Eigen::Vector2d(5,3));

    gp->resetCDT(pt_set,30.0);
    cout<<"abc"<<endl;

}


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