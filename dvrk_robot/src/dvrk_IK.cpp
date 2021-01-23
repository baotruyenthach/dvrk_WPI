/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2015-07-18

  (C) Copyright 2015-2019 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

// system
#include <iostream>
#include <fstream>
#include <map>

// cisst/saw


#include <cisstCommon/cmnPath.h>
#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstRobot/robManipulator.h>

#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnCommandLineOptions.h>
#include <cisstCommon/cmnGetChar.h>
#include <cisstCommon/cmnQt.h>
#include <cisstOSAbstraction/osaGetTime.h>
//#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitConsole.h>
//#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitConsoleQt.h>

//#include <QApplication>
//#include <QIcon>
//#include <QLocale>
//#include <clocale>

//#include <ros/ros.h>
//#include <cisst_ros_bridge/mtsROSBridge.h>
//#include <dvrk_utilities/dvrk_console.h>

using namespace std;

int main()
{
//-------Test forward kinematics------------//
    
    
      // log configuration
     cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
     cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
     cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
     cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

     cmnPath path;


     std::cout << path.GetWorkingDirectory() << std::endl;
     std::cout << "\n-------------------------------------\n" << std::endl;

 #if CISST_HAS_JSON

     // open psm-Bao.json config file
     std::string fileName("src/dvrk_env/dvrk_robot/psm-Bao.json");
     std::ifstream robotConfigFile(fileName.c_str());

     // load config file with JSON parser
     Json::Reader jreader;
     Json::Value  robotConfig;
     bool rc = jreader.parse(robotConfigFile, robotConfig);
 //    if (!rc) {
 //        std::cerr << jreader.getFormattedErrorMessages() << std::endl;
 //        return -1;
 //    }
    
     std::string content( (std::istreambuf_iterator<char>(robotConfigFile) ),
                        (std::istreambuf_iterator<char>()    ) );
     Json::FastWriter fastWriter;
     std::string output = fastWriter.write(robotConfig);
    
     std::cout << content << std::endl;   
     std::cout << "\n-------------------------------------\n" << std::endl;
     std::cout << output << std::endl; 
     std::cout << "\n-------------------------------------\n" << std::endl;
    
    
     // robManipulator with json.rob file
     robManipulator robotJson;
     robotJson.LoadRobot(robotConfig);



     // joint position all zero
     vctDoubleVec q(robotJson.links.size(), 0.0);
     std::cout << robotJson.links.size() << std::endl;
//     q[0] = 0.78539;
//     q[1] = 0;
//     q[2] = 0.1479;
//     q[3] = 0;
//     q[4] = 0;
//     q[5] = 0;
    
//     q[0] = 0.5236;
//     q[1] = 0.5236;
//     q[2] = 0.1;
//     q[3] = 0.1479;
//     q[4] = 0.78539;
//     q[5] = 1.0472;
    
 //    q[0] = 0.5236;
 //    q[1] = 0;
 //    q[2] = 0;
 //    q[3] = 0;
 //    q[4] = 0;
 //    q[5] = 0;
    
     q[0] = 0.828887;
     q[1] = 0.0;
     q[2] = 0.163596;
     q[3] = 1.57080;
     q[4] = 2.39969;
     q[5] = -1.57079;
    
     std::cout << "\n------------------End-------------------\n" << std::endl;
     std::cout << robotJson.ForwardKinematics(q) << std::endl;
   
     std::cout << "\n------------------Base-------------------\n" << std::endl;
    
     std::cout << robotJson.ForwardKinematics(q,0) << std::endl;              
     std::cout << "\n------------------1-------------------\n" << std::endl;
    
     std::cout << robotJson.ForwardKinematics(q,1) << std::endl;              
     std::cout << "\n------------------2-------------------\n" << std::endl;
    
     std::cout << robotJson.ForwardKinematics(q,2) << std::endl;              
     std::cout << "\n------------------3-------------------\n" << std::endl;
    
     std::cout << robotJson.ForwardKinematics(q,3) << std::endl;              
     std::cout << "\n------------------4-------------------\n" << std::endl;
    
     std::cout <<  robotJson.ForwardKinematics(q,4) << std::endl;              
     std::cout << "\n------------------5-------------------\n" << std::endl;
    
     std::cout <<  robotJson.ForwardKinematics(q,5) << std::endl;              
     std::cout << "\n-------------------------------------\n" << std::endl;
     
     vctDynamicVector<double> q_test( 6, 0.0 );
//     q_test[0] = 0.78539;
//     q_test[1] = 0;
//     q_test[2] = 0.1;
//     q_test[3] = 0;
//     q_test[4] = 0;
//     q_test[5] = 0;   
     
     
//     vctFrame4x4<double> Rtq =  robotJson.ForwardKinematics(q);
     vctMatrixRotation3<double> Rb1( 1.0, 0.0,  0.0,
				      0.0,  1.0, 0.0,
				     0.0,  0.0,  1.0,
				      VCT_NORMALIZE );
     vctFixedSizeVector<double,3> tb1(1000, 0.0, -0.1);
     vctFrame4x4<double> Rtq( Rb1, tb1 );

     
     std::cout << "\n------------------inverse kinematics-------------------\n" << std::endl;
     std::cout << robotJson.InverseKinematics( q_test, Rtq, 1e-2) << std::endl;
     std::cout << q_test << std::endl;
    
//     std::cout << "\n---------------Goal----------------------\n" << std::endl;
//    
//    
//     q[0] = -0.78539;
//     q[1] = 0;
//     q[2] = 0.1;
//     q[3] = 0;
//     q[4] = 0;
//     q[5] = 0;
//      
//    
//     std::cout << "\n------------------End-------------------\n" << std::endl;
//     std::cout << robotJson.ForwardKinematics(q) << std::endl;
//   
//     std::cout << "\n------------------Base-------------------\n" << std::endl;
//    
//     std::cout << robotJson.ForwardKinematics(q,0) << std::endl;              
//     std::cout << "\n------------------1-------------------\n" << std::endl;
//    
//     std::cout << robotJson.ForwardKinematics(q,1) << std::endl;              
//     std::cout << "\n------------------2-------------------\n" << std::endl;
//    
//     std::cout << robotJson.ForwardKinematics(q,2) << std::endl;              
//     std::cout << "\n------------------3-------------------\n" << std::endl;
//    
//     std::cout << robotJson.ForwardKinematics(q,3) << std::endl;              
//     std::cout << "\n------------------4-------------------\n" << std::endl;
//    
//     std::cout <<  robotJson.ForwardKinematics(q,4) << std::endl;              
//     std::cout << "\n------------------5-------------------\n" << std::endl;
//    
//     std::cout <<  robotJson.ForwardKinematics(q,5) << std::endl;              
//     std::cout << "\n-------------------------------------\n" << std::endl;

    
 #endif

    return 0;
}





