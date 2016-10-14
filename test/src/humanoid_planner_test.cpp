#include <gtest/gtest.h>


#include "../../include/humanoid_planner.hpp"


using namespace std;


void createEnvironment(){




}

int argcc;
char** argvv;

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc,argv);
    argcc=argc;
    argvv=argv;
    return RUN_ALL_TESTS();
}
