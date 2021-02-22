#include "arena_path_search/jps.h"

using namespace std;
JPS::~JPS()
{
    for (int i = 0; i < POOL_SIZE_(0); i++)
        for (int j = 0; j < POOL_SIZE_(1); j++)
                delete GridNodeMap_[i][j];
}


void JPS::setEnvironment(const GridMap::Ptr& env){
    this->grid_map_ = env;
    grid_map_->getRegion(occ_map_origin_, occ_map_size_2d_);
}

void JPS::setSearchMap(const Eigen::Vector2i pool_size, double step_size, double lamda_heu, bool use_jps ){
    
    /* set node pool & index */
    POOL_SIZE_ = pool_size;
    index_bound_min_=Eigen::Vector2i::Zero();
    index_bound_max_=POOL_SIZE_;

    GridNodeMap_ = new GridNodePtr *[POOL_SIZE_(0)];
    for (int i = 0; i < POOL_SIZE_(0); i++)
    {
        GridNodeMap_[i] = new GridNodePtr [POOL_SIZE_(1)];
        for (int j = 0; j < POOL_SIZE_(1); j++)
        {
            GridNodeMap_[i][j] = new GridNode;
        }
    }

     /* set step */
    step_size_=step_size;
    inv_step_size_=1.0/step_size_;     

    /* set lamda_heu */
    lambda_heu_=lamda_heu;

    /* set use_jps */
    use_jps_=use_jps;
}

bool JPS::ConvertToIndexAndAdjustStartEndPoints( Eigen::Vector2d start_pt,  Eigen::Vector2d end_pt, Eigen::Vector2i &start_idx, Eigen::Vector2i &end_idx)
{
    if (!Pos2Index(start_pt, start_idx) || !Pos2Index(end_pt, end_idx))
        return false;

    if (isOccupied(Index2Pos(start_idx)))
    {
        ROS_WARN("Start point is insdide an obstacle.");
        do
        {
            start_pt = (start_pt - end_pt).normalized() * step_size_ + start_pt;
            if (!Pos2Index(start_pt, start_idx))
                return false;
        } while (isOccupied(Index2Pos(start_idx)));
    }

    if (isOccupied(Index2Pos(end_idx)))
    {
        ROS_WARN("End point is insdide an obstacle.");
        do
        {
            end_pt = (end_pt - start_pt).normalized() * step_size_ + end_pt;
            //ROS_WARN_STREAM("STILL in obstacle"<<end_pt);
            if (!Pos2Index(end_pt, end_idx))
                return false;
        } while (isOccupied(Index2Pos(end_idx)));
    }    

    return true;
    
}

void JPS::AstarGetSucc(GridNodePtr currentPtr, std::vector<GridNodePtr> & neighborPtrSets, std::vector<double> & edgeCostSets){

    neighborPtrSets.clear();
    edgeCostSets.clear();
    
    for (int dx = -1; dx <= 1; dx++)
    {
        for (int dy = -1; dy <= 1; dy++)
        {   
            // check if same as current node
            if (dx == 0 && dy == 0)
                continue;

            Eigen::Vector2i neighborIdx;
            neighborIdx(0) = (currentPtr->index)(0) + dx;
            neighborIdx(1) = (currentPtr->index)(1) + dy;

            // check if neighbor idx is within valid index boundary range      
            if (neighborIdx(0) < 1 || neighborIdx(0) >= index_bound_max_(0) - 1 || neighborIdx(1) < 1 || neighborIdx(1) >= index_bound_max_(1) - 1 )
            {
                continue;
            }

            // check if in closeset
            GridNodePtr neighborPtr = GridNodeMap_[neighborIdx(0)][neighborIdx(1)];
            neighborPtr->index = neighborIdx;

            bool flag_explored = neighborPtr->rounds == rounds_;
            
            if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET)
            {
                continue; //in closed set.
            }

            

            // check if in collision
            if (isOccupied(Index2Pos(neighborPtr->index)))
            {
                continue;
            }
            /* add to costSet and neighborPtrSet */
            double edgeCost = sqrt(dx * dx + dy * dy);
            edgeCostSets.push_back(edgeCost);
            neighborPtrSets.push_back(neighborPtr);
        }
    }    
}

bool JPS::search( Eigen::Vector2d start_pt, Eigen::Vector2d end_pt, bool is_local){
    /* ---------- initialize param---------- */
    
    // spcify local or global search
    if(is_local){
        // local search
        index_origin_ = POOL_SIZE_ / 2;
        pos_origin_ = (start_pt + end_pt) / 2;
        //pos_bound_min_, pos_bound_max_;                    
    }else{
        // global search
        index_origin_ =Eigen::Vector2i::Zero();
        pos_origin_=occ_map_origin_;
        //pos_bound_min_, pos_bound_max_;            
    }

    ros::Time time_1 = ros::Time::now();

    // reset a new round
    ++rounds_;

    /* ---------- initialize search---------- */
    // convert start_pt,end_pt to start_id, end_id 
    Eigen::Vector2i start_idx, end_idx;
    //start_idx=Pos2Index(start_pt);
    //end_idx=Pos2Index(end_pt);
    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx))
    {
        ROS_ERROR("Unable to handle the initial or end point, force return!");
        return false;
    }

    // init openSet
    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, GridNodeComparator> empty;
    openSet_.swap(empty);

    // init current & neighbor node
    GridNodePtr currentPtr = NULL;
    GridNodePtr neighborPtr = NULL;

    // init start and end node
    GridNodePtr startPtr = GridNodeMap_[start_idx(0)][start_idx(1)];
    GridNodePtr endPtr = GridNodeMap_[end_idx(0)][end_idx(1)];

    startPtr->index = start_idx;
    startPtr->rounds = rounds_;
    startPtr->gScore = 0;
    startPtr->fScore = getHeu(startPtr, endPtr);
    startPtr->cameFrom = NULL;
    //put start in open set
    startPtr->state = GridNode::OPENSET; //put start node in open set
    openSet_.push(startPtr); 

    endPtr->index = end_idx;



    // temp vars
    double tentative_gScore;
    int num_iter = 0;
    std::vector<GridNodePtr> neighborPtrSets;
    std::vector<double> edgeCostSets; 

    /* ---------- search loop---------- */
    while (!openSet_.empty())
    {   
        // pop from OPENSET: first priority element in openSet
        num_iter++;
        currentPtr = openSet_.top();
        openSet_.pop();
        
        // check currentNode reached goal
        if (currentPtr->index(0) == endPtr->index(0) && currentPtr->index(1) == endPtr->index(1))
        {
            // ros::Time time_2 = ros::Time::now();
            // printf("\033[34mA star iter:%d, time:%.3f\033[0m\n",num_iter, (time_2 - time_1).toSec()*1000);
            // if((time_2 - time_1).toSec() > 0.1)
            //     ROS_WARN("Time consume in A star path finding is %f", (time_2 - time_1).toSec() );
            gridPath_ = retrievePath(currentPtr);
           
            return true;
        }

        
        // move to CLOSESET
        currentPtr->state = GridNode::CLOSEDSET; //move current node from open set to closed set.

        //get the succesors
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);  //STEP 4: finish AstarPathFinder::AstarGetSucc yourself     
        
        // update openSet
        for(int i = 0; i < (int)neighborPtrSets.size(); i++)
        {
            neighborPtr=neighborPtrSets[i];
            tentative_gScore=currentPtr->gScore+edgeCostSets[i];
            
            if (!(neighborPtr->rounds==rounds_))//(neighborPtr->state=GridNode::UNDEFINED)
            {   
                //discover a new node
                // set new rounds to negihborPtr
                neighborPtr->rounds = rounds_;
                neighborPtr->state = GridNode::OPENSET;
                neighborPtr->cameFrom = currentPtr;
                neighborPtr->gScore = tentative_gScore;
                neighborPtr->fScore = tentative_gScore + lambda_heu_*getHeu(neighborPtr, endPtr);
                openSet_.push(neighborPtr); //put neighbor in open set and record it.
            }
            else if (tentative_gScore < neighborPtr->gScore)
            {   //in open set and need update
                neighborPtr->cameFrom = currentPtr;
                neighborPtr->gScore = tentative_gScore;
                neighborPtr->fScore = tentative_gScore + lambda_heu_*getHeu(neighborPtr, endPtr);
            }
        }
        


        if(is_local)
        {   // should not exceed time
            ros::Time time_2 = ros::Time::now();
            if ((time_2 - time_1).toSec() > 0.2)
            {
                ROS_WARN("Failed in A star path searching !!! 0.2 seconds time limit exceeded.");
                return false;
            }
        }
    }

    ros::Time time_2 = ros::Time::now();

    if ((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in A star path finding is %.3fs, iter=%d", (time_2 - time_1).toSec(), num_iter);

    return false;

}


std::vector<GridNodePtr> JPS::retrievePath(GridNodePtr currentPtr){
    std::vector<GridNodePtr> path;
    path.push_back(currentPtr);

    while(currentPtr->cameFrom!=NULL)
    {
        currentPtr=currentPtr->cameFrom;
        path.push_back(currentPtr);
       
    }
    return path;
}


std::vector<Eigen::Vector2d> JPS::getPath()
{
    std::vector<Eigen::Vector2d> path;

    for (auto ptr : gridPath_)
        path.push_back(Index2Pos(ptr->index));

    reverse(path.begin(), path.end());
    return path;
}

