#include <iostream>
#include <sstream>

#include "edge_calib.h"
#include "g2o_optimize.h"

// If you want check your graph's vertices and edges, please uncomment this.
//#define CHECK_GRAPH
#define CLOSED_FORM

using namespace std;

int main(int argc, char** argv)
{
//     string filePathName;
// 
//     cout << "Enter the data file path name, end with Enter (the default path name is ./)." << endl;
//     
//     cin >> filePathName;
// 
//     cout << "Success, reading icp results." << endl; 
//     
//     string icpDataName, voDataName, initValueName;
//     stringstream ss;

//     ss << filePathName << "/icp.txt" << endl;
//     ifstream icpIn(ss.str().c_str());
    ifstream icpIn("/home/doom/github/CSO/data/icp.txt");
    int NumIcp = 0;
    vector<SE3Type> vIcp;
    if(icpIn.fail())
    {
        cerr << "Error reading icp file." << endl;
        return 1;
    }
    else
    {
        string stemp;
        getline(icpIn, stemp, '\n');
        while(icpIn.good()){
            NumIcp++;
            getline(icpIn, stemp, '\n');
        }
        cout << "There are " << NumIcp << " Icp results in this direcotory." << endl << endl;
        icpIn.close();
//         icpIn.open(ss.str().c_str());
	icpIn.open("/home/doom/github/CSO/data/icp.txt");
        for(int i = 0; i < NumIcp; i++)
        {
	    Eigen::Matrix<double, 7, 1> est;
	    SE3Type tempSE3;
	    for (int i=0; i<7; i++)
	      icpIn  >> est[i];
	    Eigen::Quaterniond q;
	    q.w() = est[3];
	    q.x() = est[4]; q.y() = est[7]; q.z() = est[6];
	    q.normalize();
	    tempSE3 = SE3Type(SO3Type(q), Point(est.segment(0,3)));
	    vIcp.push_back(tempSE3);
        }
        icpIn.close();
    }
    
    cout << "Success, reading vo results." << endl; 
//     ss.str("");
//     ss << filePathName << "/vo.txt" << endl;
//     ifstream voIn(ss.str().c_str());
    ifstream voIn("/home/doom/github/CSO/data/vo.txt");
    int NumVo = 0;
    vector<SE3Type> vVo;
    if(voIn.fail())
    {
        cerr << "Error reading vo file." << endl;
        return 1;
    }
    else
    {
        string stemp;
        getline(voIn, stemp, '\n');
        while(voIn.good()){
            NumVo++;
            getline(voIn, stemp, '\n');
        }
        cout << "There are " << NumVo << " Vo results in this direcotory." << endl << endl;
        voIn.close();
//         voIn.open(ss.str().c_str());
	voIn.open("/home/doom/github/CSO/data/vo.txt");
        for(int i = 0; i < NumVo; i++)
        {
	    Eigen::Matrix<double, 7, 1> est;
	    SE3Type tempSE3;
	    for (int i=0; i<7; i++)
	      voIn  >> est[i];
	    Eigen::Quaterniond q;
	    q.w() = est[3];
	    q.x() = est[4]; q.y() = est[7]; q.z() = est[6];
	    q.normalize();
	    tempSE3 = SE3Type(SO3Type(q), Point(est.segment(0,3)));
	    vVo.push_back(tempSE3);
        }
        voIn.close();
    }
    
    cout << "Success, reading init value." << endl; 
//     ss.str("");
//     ss << filePathName << "/initValue.txt" << endl;
//     ifstream initIn(ss.str().c_str());
    ifstream initIn("/home/doom/github/CSO/data/initValue.txt");
    SE3Type initSE3;
    Matrix6D initHessian;
    if(initIn.fail())
    {
        cerr << "Error reading init value file." << endl;
        return 1;
    }
    else
    {
	Eigen::Matrix<double, 7, 1> est;
	for(int i = 0; i < 7; i++)
	  initIn >> est[i];
	Eigen::Quaterniond q;
	q.w() = est[3];
	q.x() = est[4]; q.y() = est[7]; q.z() = est[6];
	q.normalize();
	initSE3 = SE3Type(SO3Type(q), Point(est.segment(0,3)));
	Vector6D vinfo;
	for(int i = 0; i < 6; i++)
	  initIn >> vinfo[i];
	initHessian = vinfo.asDiagonal();
        initIn.close();
    }
    cout << "Finish reading all available input files." << endl;

    // construct a new optimizer
    SparseOptimizer optimizer;
    Optimizer::initOptimizer(optimizer);
    
    Matrix6D I6X6 = Matrix6D::Identity();
    SE3Type calibEst;
    Matrix6D calibInfo;
    for(int i = 0; i < NumIcp; i++)
    {
	if(i == 0)
	{
	    Optimizer::addVertexSE3(optimizer, initSE3, 0);
	    Optimizer::addEdgeCalib(optimizer, 0, vVo[i], vIcp[i], I6X6);
	    Optimizer::addEdgePrior(optimizer, 0, initSE3, initHessian);
	    
	    Optimizer::optimize(optimizer, 20);
	    
	    calibEst = Optimizer::getEstimation(optimizer);
	    calibInfo = Optimizer::getMargInfo(optimizer);
	    
	    Optimizer::removeEdges(optimizer);
	}
// 	addVertexSE3(optimizer, calibEst, 0);
	Optimizer::addEdgeCalib(optimizer, 0, vVo[i], vIcp[i], I6X6);
	Optimizer::addEdgePrior(optimizer, 0, calibEst, calibInfo);
	
	Optimizer::optimize(optimizer, 20);
	
	calibEst = Optimizer::getEstimation(optimizer);
	calibInfo = Optimizer::getMargInfo(optimizer);
	
	Optimizer::removeEdges(optimizer);
    }
    
    cout << "The calibration result is " << calibEst.translation() << endl;

    return 0;
}
