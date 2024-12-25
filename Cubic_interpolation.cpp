// 使用了Eigen

void mergeMotionPlans(std::vector<moveit::planning_interface::MoveGroupInterface::Plan>& motionPlans, moveit::planning_interface::MoveGroupInterface::Plan& mergedPlanResult, float timeScale){
    int jointNum;
    int pointsNumOfMergedMotion=0;
    double planningTime = 0;
    moveit::planning_interface::MoveGroupInterface::Plan *motionPlanPointer;

    for(int planIndex=0;planIndex<motionPlans.size();planIndex++){
        motionPlanPointer = &(motionPlans.at(planIndex));
        pointsNumOfMergedMotion+=motionPlanPointer->trajectory_.joint_trajectory.points.size();
        planningTime+=motionPlanPointer->planning_time_;
        jointNum=motionPlanPointer->trajectory_.joint_trajectory.joint_names.size();

//        if(motionPlans.size()>1){
//            ROS_INFO("********************************** %d th plan to merge *************************************",planIndex);
//            printMotionPlanInfo(motionPlanPointer);
//        }
    }
    pointsNumOfMergedMotion = pointsNumOfMergedMotion - motionPlans.size()+1;

    double *positionsOfMergedMotion=(double *) malloc(sizeof(double)*jointNum*pointsNumOfMergedMotion);
    double *velocitiesOfMergedMotion=(double *) malloc(sizeof(double)*jointNum*pointsNumOfMergedMotion);
    double *accelerationsOfMergedMotion =(double *) malloc(sizeof(double)*jointNum*pointsNumOfMergedMotion);
    double *timeStampsOfMergedMotion =(double *) malloc(sizeof(double)*pointsNumOfMergedMotion);

    // get trajectory data for sequential motion plans
    int currentPointIndex=0;
    double timeOffset =0;
    int currentPathLen;
    for(int planIndex=0;planIndex<motionPlans.size();planIndex++){
        motionPlanPointer = &(motionPlans.at(planIndex));
        currentPathLen=motionPlanPointer->trajectory_.joint_trajectory.points.size();
        for(int i=0;i<currentPathLen-1;i++){
            for(int j=0;j<jointNum;j++)
                positionsOfMergedMotion[jointNum*currentPointIndex+j] = motionPlanPointer->trajectory_.joint_trajectory.points[i].positions[j];
            timeStampsOfMergedMotion[currentPointIndex] = motionPlanPointer->trajectory_.joint_trajectory.points[i].time_from_start.toSec()+timeOffset;
            currentPointIndex++;
        }
        timeOffset+=motionPlanPointer->trajectory_.joint_trajectory.points[currentPathLen-1].time_from_start.toSec();
    }

    // get the last point of the last motion plan trajectory
    for(int j=0;j<jointNum;j++)
        positionsOfMergedMotion[jointNum*currentPointIndex+j] = motionPlanPointer->trajectory_.joint_trajectory.points[currentPathLen-1].positions[j];
    timeStampsOfMergedMotion[currentPointIndex] = timeOffset;

    // scale the velocity
    for(int i=0;i<pointsNumOfMergedMotion;i++)
        timeStampsOfMergedMotion[i]=timeStampsOfMergedMotion[i]*timeScale;

    // merged motion smoothing
    Eigen::MatrixXd A[6];
    Eigen::MatrixXd b[6];
    Eigen::MatrixXd x[6];
    double currentT;
    int cubicSpineSegmentNum = pointsNumOfMergedMotion-1;
    for(int i=0;i<jointNum;i++){
        A[i].resize(4*cubicSpineSegmentNum,4*cubicSpineSegmentNum);
        b[i].resize(4*cubicSpineSegmentNum,1);
        x[i].resize(4*cubicSpineSegmentNum,1);

        A[i].setZero();
        b[i].setZero();
        x[i].setZero();
    }
    // constraint at the first time point
    currentT = timeStampsOfMergedMotion[0];
    for(int i=0;i<jointNum;i++){
        A[i].matrix()(0,0)=currentT*currentT*currentT;
        A[i].matrix()(0,1)=currentT*currentT;
        A[i].matrix()(0,2)=currentT;
        A[i].matrix()(0,3)=1;

        b[i].matrix()(0,0)=positionsOfMergedMotion[i];

        A[i].matrix()(1,0)=3*currentT*currentT;
        A[i].matrix()(1,1)=2*currentT;
        A[i].matrix()(1,2)=1;

        b[i].matrix()(1,0)=0;
    }
    // constraint for time point between the first and the last one
    for(int i=2;i<=(pointsNumOfMergedMotion-1);i++){
        currentT = timeStampsOfMergedMotion[i-1];

        for(int j=0;j<jointNum;j++){
            // left position limit of i th time point
            A[j].matrix()((i-2)*4+2,(i-2)*4+0)=currentT*currentT*currentT;
            A[j].matrix()((i-2)*4+2,(i-2)*4+1)=currentT*currentT;
            A[j].matrix()((i-2)*4+2,(i-2)*4+2)=currentT;
            A[j].matrix()((i-2)*4+2,(i-2)*4+3)=1;

            b[j].matrix()((i-2)*4+2,0)=positionsOfMergedMotion[(i-1)*jointNum+j];

            // right position limit of i th time point
            A[j].matrix()((i-2)*4+2+1,0+(i-2)*4+4)=currentT*currentT*currentT;
            A[j].matrix()((i-2)*4+2+1,1+(i-2)*4+4)=currentT*currentT;
            A[j].matrix()((i-2)*4+2+1,2+(i-2)*4+4)=currentT;
            A[j].matrix()((i-2)*4+2+1,3+(i-2)*4+4)=1;

            b[j].matrix()((i-2)*4+2+1,0)=positionsOfMergedMotion[(i-1)*jointNum+j];


            // equal speed constraint of i th time point
            A[j].matrix()((i-2)*4+2+2,0+(i-2)*4)=3*currentT*currentT;
            A[j].matrix()((i-2)*4+2+2,1+(i-2)*4)=2*currentT;
            A[j].matrix()((i-2)*4+2+2,2+(i-2)*4)=1;

            A[j].matrix()((i-2)*4+2+2,0+(i-2)*4+4)=-3*currentT*currentT;
            A[j].matrix()((i-2)*4+2+2,1+(i-2)*4+4)=-2*currentT;
            A[j].matrix()((i-2)*4+2+2,2+(i-2)*4+4)=-1;


            //equal acceleration constraint of i th time point
            A[j].matrix()((i-2)*4+2+3,0+(i-2)*4)=6*currentT;
            A[j].matrix()((i-2)*4+2+3,1+(i-2)*4)=2;

            A[j].matrix()((i-2)*4+2+3,0+(i-2)*4+4)=-6*currentT;
            A[j].matrix()((i-2)*4+2+3,1+(i-2)*4+4)=-2;
        }
    }

    // constraint for the last time point
    currentT = timeStampsOfMergedMotion[pointsNumOfMergedMotion-1];
    for(int i=0;i<jointNum;i++){
        A[i].matrix()(4*cubicSpineSegmentNum-2,4*cubicSpineSegmentNum-4)=currentT*currentT*currentT;
        A[i].matrix()(4*cubicSpineSegmentNum-2,4*cubicSpineSegmentNum-3)=currentT*currentT;
        A[i].matrix()(4*cubicSpineSegmentNum-2,4*cubicSpineSegmentNum-2)=currentT;
        A[i].matrix()(4*cubicSpineSegmentNum-2,4*cubicSpineSegmentNum-1)=1;

        b[i].matrix()(4*cubicSpineSegmentNum-2,0)=positionsOfMergedMotion[(pointsNumOfMergedMotion-1)*jointNum+i];


        A[i].matrix()(4*cubicSpineSegmentNum-1,4*cubicSpineSegmentNum-4)=3*currentT*currentT;
        A[i].matrix()(4*cubicSpineSegmentNum-1,4*cubicSpineSegmentNum-3)=2*currentT;
        A[i].matrix()(4*cubicSpineSegmentNum-1,4*cubicSpineSegmentNum-2)=1;

        b[i].matrix()(4*cubicSpineSegmentNum-1,0)=0;
    }

    // compute velocity and acceleration
    // the first time point
    currentT = timeStampsOfMergedMotion[0];
    for(int i=0;i<jointNum;i++){
        x[i] = A[i].inverse()*b[i];
        velocitiesOfMergedMotion[i]=0;
        accelerationsOfMergedMotion[i]=6*currentT * x[i].matrix()(0,0)+ 2 * x[i].matrix()(1,0);
//        std::cout<<"A["<<i<<"]: "<<std::endl;
//        std::cout<<A[i]<<std::endl;

//        std::cout<<"A["<<i<<"] inverse: "<<std::endl;
//        std::cout<<A[i].inverse()<<std::endl;

//        std::cout<<"x["<<i<<"]: "<<std::endl;
//        std::cout<<x[i]<<std::endl;

//        std::cout<<"b["<<i<<"]: "<<std::endl;
//        std::cout<<b[i]<<std::endl;

    }
    // time point between the first one and the last one.
    for(int i=2;i<=(pointsNumOfMergedMotion-1);i++){
        currentT = timeStampsOfMergedMotion[i-1];

        for(int j=0;j<jointNum;j++){
            velocitiesOfMergedMotion[(i-1)*jointNum+j] =3*currentT*currentT * x[j].matrix()((i-2)*4+0,0)+
                                                               2*currentT * x[j].matrix()((i-2)*4+1,0)+
                                                               1 * x[j].matrix()((i-2)*4+2,0);

            accelerationsOfMergedMotion[(i-1)*jointNum+j]=6*currentT * x[j].matrix()((i-2)*4+0,0)+
                                                                  2 * x[j].matrix()((i-2)*4+1,0);
        }
    }
    // the last time point
    currentT = timeStampsOfMergedMotion[pointsNumOfMergedMotion-1];
    for(int i=0;i<jointNum;i++){
        velocitiesOfMergedMotion[(pointsNumOfMergedMotion-1)*jointNum+i]=3*currentT*currentT * x[i].matrix()(4*cubicSpineSegmentNum-4,0)+
                                                                                2*currentT * x[i].matrix()(4*cubicSpineSegmentNum-3,0)+
                                                                                1 * x[i].matrix()(4*cubicSpineSegmentNum-2,0);

        accelerationsOfMergedMotion[(pointsNumOfMergedMotion-1)*jointNum+i]=6*currentT * x[i].matrix()(4*cubicSpineSegmentNum-4,0)+
                                                                                   2 * x[i].matrix()(4*cubicSpineSegmentNum-3,0);
    }

    // set interplated data to merged trajectory
    mergedPlanResult.trajectory_.joint_trajectory.points.resize(pointsNumOfMergedMotion);
    for(int i=0;i<pointsNumOfMergedMotion;i++)
    {
        mergedPlanResult.trajectory_.joint_trajectory.points[i].positions.resize(jointNum);
        mergedPlanResult.trajectory_.joint_trajectory.points[i].velocities.resize(jointNum);
        mergedPlanResult.trajectory_.joint_trajectory.points[i].accelerations.resize(jointNum);
        for(int j=0;j<jointNum;j++){
            mergedPlanResult.trajectory_.joint_trajectory.points[i].positions[j] = positionsOfMergedMotion[i*jointNum+j];
            mergedPlanResult.trajectory_.joint_trajectory.points[i].velocities[j] = velocitiesOfMergedMotion[i*jointNum+j];
            mergedPlanResult.trajectory_.joint_trajectory.points[i].accelerations[j] = accelerationsOfMergedMotion[i*jointNum+j];
        }
        mergedPlanResult.trajectory_.joint_trajectory.points[i].time_from_start.fromSec(timeStampsOfMergedMotion[i]);
    }

    mergedPlanResult.start_state_=motionPlans.at(0).start_state_;
    mergedPlanResult.planning_time_=planningTime;
    mergedPlanResult.trajectory_.joint_trajectory.joint_names=motionPlans.at(0).trajectory_.joint_trajectory.joint_names;
    mergedPlanResult.trajectory_.joint_trajectory.header=motionPlans.at(0).trajectory_.joint_trajectory.header;

    if(motionPlans.size()>1){
        ROS_INFO("********************************** the merged motion plan is:*************************************");
        //printMotionPlanInfo(&mergedPlanResult);
    }

    free(positionsOfMergedMotion);
    free(velocitiesOfMergedMotion);
    free(accelerationsOfMergedMotion);
    free(timeStampsOfMergedMotion);
}


void sCurveRation0_1Plan(double motionTimeLength, int samplingPointNum, std::vector<double>& positionRatio){
    positionRatio.clear();
    positionRatio.resize(samplingPointNum);
    positionRatio.at(0)=0;
    positionRatio.at(samplingPointNum-1)=1.0;

    double jerTimeLimit = 0.2;
    double accTimeLimit = 0.3;

    if(motionTimeLength<2*jerTimeLimit){
        //p(t) = 1/6*jerk*t^3

        double jerk = 0.5*6*8/(motionTimeLength*motionTimeLength*motionTimeLength);
        for(int i=1;i<=(samplingPointNum-1)/2;i++){
            double currentT = i*1.0/(samplingPointNum-1)*motionTimeLength;

            positionRatio.at(i) = 1.0/6*jerk*currentT*currentT*currentT;
            positionRatio.at(samplingPointNum-1-i)= 1 - positionRatio.at(i);
        }
    }
    else
    {
        if(motionTimeLength<2*(accTimeLimit+jerTimeLimit)){
            //t_jl=0.2;
            //a(t_jl) = jerk*t_jl
            //v(t_jl) = 1/2*jerk*t_jl^2
            //p(t_jl) = 1/6*jerk*t_jl^3

            //p(t) = p(t_jl) + v(t_jl)*(t-t_jl) + 1/2*a(t_jl)*(t-t_jl)^2
            //p(t) = 1/6*jerk*t_jl^3 + 1/2*jerk*t_jl^2*(t-t_jl) + 1/2*jerk*t_jl*(t-t_jl)^2

            //p(t)=0.5
            double jerk = 0.5/(1.0/6*jerTimeLimit*jerTimeLimit*jerTimeLimit +
                                 1.0/2*jerTimeLimit*jerTimeLimit*(motionTimeLength/2 - jerTimeLimit) +
                                 1.0/2*jerTimeLimit*(motionTimeLength/2 - jerTimeLimit)*(motionTimeLength/2 - jerTimeLimit));

            for(int i=1;i<=(samplingPointNum-1)/2;i++){
                double currentT = i*1.0/(samplingPointNum-1)*motionTimeLength;

                if(currentT<jerTimeLimit)
                    positionRatio.at(i) = 1.0/6*jerk*currentT*currentT*currentT;
                else
                    positionRatio.at(i) = 1.0/6*jerk*jerTimeLimit*jerTimeLimit*jerTimeLimit +
                                          1.0/2*jerk*jerTimeLimit*jerTimeLimit*(currentT-jerTimeLimit) +
                                          1.0/2*jerk*jerTimeLimit*(currentT-jerTimeLimit)*(currentT-jerTimeLimit);
                positionRatio.at(samplingPointNum-1-i)= 1 - positionRatio.at(i);
            }
        }
        else {
            //t_jl=0.2;
            //a(t_jl) = jerk*t_jl
            //v(t_jl) = 1/2*jerk*t_jl^2
            //p(t_jl) = 1/6*jerk*t_jl^3

            //t_al=0.3;
            //a(t) = 0;
            //v(t) = 1/2*jerk*t_jl^2 + jerk*t_jl*t_al

            //p(t) = p(t_jl) + v(t_jl)*t_al + 1/2*a(t_jl)*t_al^2 + v(t)(t-t_jl-t_al)
            //p(t) = 1/6*jerk*t_jl^3 + 1/2*jerk*t_jl^2*t_al + 1/2*jerk*t_jl*t_al^2 + (1/2*jerk*t_jl^2 + jerk*t_jl*t_al)*(t-t_jl-t_al)

            double jerk = 0.5/(1.0/6*jerTimeLimit*jerTimeLimit*jerTimeLimit +
                                 1.0/2*jerTimeLimit*jerTimeLimit*accTimeLimit +
                                 1.0/2*jerTimeLimit*accTimeLimit*accTimeLimit +
                                 (1.0/2*jerTimeLimit*jerTimeLimit + jerTimeLimit*accTimeLimit)*(motionTimeLength/2-jerTimeLimit-accTimeLimit));

            for(int i=1;i<=(samplingPointNum-1)/2;i++){
                double currentT = i*1.0/(samplingPointNum-1)*motionTimeLength;

                if(currentT<jerTimeLimit)
                    positionRatio.at(i) = 1.0/6*jerk*currentT*currentT*currentT;
                else
                    if(currentT<(jerTimeLimit+accTimeLimit))
                        positionRatio.at(i) = 1.0/6*jerk*jerTimeLimit*jerTimeLimit*jerTimeLimit +
                                              1.0/2*jerk*jerTimeLimit*jerTimeLimit*(currentT-jerTimeLimit) +
                                              1.0/2*jerk*jerTimeLimit*(currentT-jerTimeLimit)*(currentT-jerTimeLimit);
                    else
                        positionRatio.at(i) = 1.0/6*jerk*jerTimeLimit*jerTimeLimit*jerTimeLimit +
                                              1.0/2*jerk*jerTimeLimit*jerTimeLimit*accTimeLimit +
                                              1.0/2*jerk*jerTimeLimit*accTimeLimit*accTimeLimit+
                                              jerk*(1.0/2*jerTimeLimit*jerTimeLimit + jerTimeLimit*accTimeLimit)*(currentT-jerTimeLimit-accTimeLimit);
                positionRatio.at(samplingPointNum-1-i)= 1 - positionRatio.at(i);
            }
        }
    }
}


