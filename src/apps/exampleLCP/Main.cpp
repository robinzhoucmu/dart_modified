#include "MyWindow.h"
#include "dynamics/BodyNodeDynamics.h"
#include "dynamics/SkeletonDynamics.h"
#include "kinematics/FileInfoSkel.hpp"
#include "utils/Paths.h"



using namespace std;
using namespace Eigen;
using namespace kinematics;
using namespace dynamics;

int main(int argc, char* argv[])
{
    FileInfoSkel<SkeletonDynamics> model, model2;
    model.loadFile(DART_DATA_PATH"/skel/ground.skel", kinematics::SKEL);
    model2.loadFile(DART_DATA_PATH"/skel/cube.skel", kinematics::SKEL);

    MyWindow window((SkeletonDynamics*)model.getSkel(), (SkeletonDynamics*)model2.getSkel());
   
    cout << model.getSkel()->getNumDofs() << endl;
    cout << model2.getSkel()->getNumDofs() << endl;
    glutInit(&argc, argv);
    window.initWindow(640, 480, "expForwardDyn");
    glutMainLoop();

    return 0;
}